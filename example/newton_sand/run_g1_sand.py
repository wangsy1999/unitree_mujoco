"""G1 pose-PD landing on MPM sand, with staggered two-way force feedback.

Coupling follows Newton 1.7's example_mpm_twoway_coupling.py (Apache-2.0):
https://github.com/newton-physics/newton/blob/v1.7.0/newton/examples/mpm/example_mpm_twoway_coupling.py
No ONNX, DDS, or Isaac Sim is used by this standalone material demo.
"""
import argparse
import json
from pathlib import Path
import sys
import time

import mujoco
import numpy as np
import newton
import warp as wp
from newton.solvers import SolverImplicitMPM

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'simulate_python'))
from physics_backend import create_backend


@wp.kernel
def sand_reaction(dt: float, ids: wp.array[int], impulses: wp.array[wp.vec3],
                  positions: wp.array[wp.vec3], collider_bodies: wp.array[int],
                  body_q: wp.array[wp.transform], com: wp.array[wp.vec3],
                  forces: wp.array[wp.spatial_vector]):
    i = wp.tid()
    cid = ids[i]
    if cid >= 0 and cid < collider_bodies.shape[0]:
        body = collider_bodies[cid]
        if body >= 0:
            force = impulses[i] / dt
            lever = positions[i] - wp.transform_point(body_q[body], com[body])
            wp.atomic_add(forces, body, wp.spatial_vector(force, wp.cross(lever, force)))


@wp.kernel
def prepare_colliders(dt: float, q: wp.array[wp.transform], v: wp.array[wp.spatial_vector],
                      sand_force: wp.array[wp.spatial_vector], inv_mass: wp.array[float],
                      inv_inertia: wp.array[wp.mat33], out_q: wp.array[wp.transform],
                      out_v: wp.array[wp.spatial_vector]):
    # Remove the lagged reaction from the predicted collider velocity, as in the
    # official explicit coupling example. This is an approximate articulated coupling.
    body = wp.tid()
    force = sand_force[body]
    rotation = wp.transform_get_rotation(q[body])
    dv = dt * inv_mass[body] * wp.spatial_top(force)
    dw = dt * wp.quat_rotate(rotation, inv_inertia[body] *
                            wp.quat_rotate_inv(rotation, wp.spatial_bottom(force)))
    out_q[body] = q[body]
    out_v[body] = v[body] - wp.spatial_vector(dv, dw)


@wp.kernel
def add_reaction(sand_force: wp.array[wp.spatial_vector], forces: wp.array[wp.spatial_vector]):
    body = wp.tid()
    forces[body] = forces[body] + sand_force[body]


def load_config(path):
    cfg = json.loads(path.read_text())
    lower, upper = np.asarray(cfg['bed_lower']), np.asarray(cfg['bed_upper'])
    numeric = np.r_[lower, upper, [cfg[k] for k in (
        'voxel_size', 'particles_per_cell', 'density', 'friction', 'young_modulus',
        'poisson_ratio', 'frame_dt', 'spawn_height', 'settle_seconds')]]
    if lower.shape != (3,) or upper.shape != (3,) or not np.isfinite(numeric).all():
        raise ValueError('sand parameters must be finite; bounds must have three coordinates')
    if (upper <= lower).any() or any(cfg[k] <= 0 for k in (
            'voxel_size', 'particles_per_cell', 'density', 'young_modulus', 'frame_dt')):
        raise ValueError('bed dimensions, resolution, density, stiffness and timestep must be positive')
    if cfg['particles_per_cell'] < 2:
        raise ValueError('this demo requires at least 2 particles per cell for sand contact sampling')
    if not 0 <= cfg['poisson_ratio'] < .5 or cfg['friction'] < 0 or cfg['settle_seconds'] < 0:
        raise ValueError('invalid material or settling parameters')
    for key in ('max_iterations', 'rigid_substeps'):
        if type(cfg[key]) is not int or cfg[key] < 1:
            raise ValueError(f'{key} must be a positive integer')
    # Keep the substrate consistent with the configured bottom; it is the only world geom.
    model = mujoco.MjModel.from_xml_path(str(ROOT / cfg['scene']))
    substrate = model.geom('sand_substrate')
    bottom = substrate.pos[2] + substrate.size[2]
    if not np.isclose(lower[2], bottom):
        raise ValueError('bed_lower.z must match sand_substrate height in the scene XML')
    resolution = np.ceil((upper - lower) * cfg['particles_per_cell'] / cfg['voxel_size']).astype(int)
    if np.prod(resolution.astype(np.int64)) > 500000:
        raise ValueError('demo particle limit is 500000; reduce bed dimensions or resolution')
    return cfg, model, lower, upper, resolution


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('g1_sand.json'))
    parser.add_argument('--device', default='cuda:0', help='cuda:0 recommended; cpu is for small diagnostics')
    parser.add_argument('--seconds', type=float, default=10.)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--sand-view', choices=['surface', 'particles'], default='surface',
                        help='smooth sand surface (default) or diagnostic particle dots')
    parser.add_argument('--check-config', action='store_true', help='validate scene and particle budget only')
    args = parser.parse_args()
    if not np.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be finite and positive')
    cfg, host, lower, upper, resolution = load_config(args.config)
    print(f'Sand bed {upper-lower} m; {np.prod(resolution)} particles; voxel {cfg["voxel_size"]} m', flush=True)
    if args.check_config:
        print('Config and G1 scene OK; no simulation was run')
        return
    wp.init()
    surface = None
    if not args.headless and args.sand_view == 'surface':
        from sand_surface import create_surface_scene
        host, surface = create_surface_scene(ROOT / cfg['scene'], lower, upper,
                                              (upper - lower) / resolution)
    with wp.ScopedDevice(args.device):
        run(args, cfg, host, lower, upper, resolution, surface)


def run(args, cfg, host, lower, upper, resolution, surface=None):
    robot_cfg = json.loads((ROOT / cfg['robot_config']).read_text())
    joints = np.array([host.joint(name).id for name in robot_cfg['joint_names']])
    qadr, vadr = host.jnt_qposadr[joints], host.jnt_dofadr[joints]
    motors = []
    for joint in joints:
        matches = np.flatnonzero((host.actuator_trnid[:, 0] == joint) &
                                (host.actuator_trntype == mujoco.mjtTrn.mjTRN_JOINT))
        if len(matches) != 1:
            raise ValueError('expected one direct motor per controlled G1 joint')
        motors.append(int(matches[0]))
    motors = np.asarray(motors)
    if not np.allclose(host.actuator_gear[motors], np.tile([1, 0, 0, 0, 0, 0], (len(motors), 1))):
        raise ValueError('expected unit-gear G1 motors')
    target = np.asarray(robot_cfg['joint_sign']) * robot_cfg['default_joint_pos']
    kp, kd = np.asarray(robot_cfg['kp']), np.asarray(robot_cfg['kd'])
    limits = np.asarray(robot_cfg['effort_limit'])
    dt = cfg['frame_dt']
    host.opt.timestep = dt / cfg['rigid_substeps']
    host.dof_armature[vadr] = robot_cfg['armature']
    host.dof_damping[vadr] = 0
    host.dof_frictionloss[vadr] = 0
    data = mujoco.MjData(host)
    data.qpos[qadr] = target
    data.qpos[:3] = [0, 0, cfg['spawn_height']]
    data.qpos[3:7] = [1, 0, 0, 0]
    mujoco.mj_forward(host, data)
    robot = create_backend('newton', host, data, ROOT / cfg['scene'], args.device)
    robot.prepare()
    robot._push()
    model = robot.engine_model
    # MJCF import is configured for rigid contacts. Explicitly enable particle
    # collision on those same collision shapes, excluding visual copies/sites.
    flags = model.shape_flags.numpy()
    collisions = (flags & int(newton.ShapeFlags.COLLIDE_SHAPES)) != 0
    flags[collisions] |= int(newton.ShapeFlags.COLLIDE_PARTICLES)
    flags[~collisions] &= ~int(newton.ShapeFlags.COLLIDE_PARTICLES)
    model.shape_flags.assign(flags)

    builder = newton.ModelBuilder()
    SolverImplicitMPM.register_custom_attributes(builder)
    spacing = (upper - lower) / resolution
    builder.add_particle_grid(
        pos=wp.vec3(*(lower + spacing * .5)), rot=wp.quat_identity(), vel=wp.vec3(0.),
        dim_x=int(resolution[0]), dim_y=int(resolution[1]), dim_z=int(resolution[2]),
        cell_x=float(spacing[0]), cell_y=float(spacing[1]), cell_z=float(spacing[2]),
        mass=float(np.prod(spacing) * cfg['density']), jitter=0., radius_mean=float(min(spacing) * .5),
        custom_attributes={'mpm:friction': cfg['friction'], 'mpm:young_modulus': cfg['young_modulus'],
                           'mpm:poisson_ratio': cfg['poisson_ratio']})
    sand_model = builder.finalize(device=args.device)
    options = SolverImplicitMPM.Config()
    options.voxel_size = cfg['voxel_size']
    options.grid_type = 'fixed'
    options.grid_padding = 8
    options.max_active_cell_count = 1 << 15
    options.strain_basis = 'P0'
    options.max_iterations = cfg['max_iterations']
    options.critical_fraction = 0.
    sand = SolverImplicitMPM(sand_model, config=options)
    sand.setup_collider(model=model, body_q=robot.state.body_q)
    collider_bodies = sand.collider_body_index.numpy()
    if -1 not in collider_bodies or len(collider_bodies) < 2:
        raise RuntimeError('MPM requires both the substrate and robot colliders')
    print(f'MPM colliders: {len(collider_bodies)} (substrate + robot bodies)', flush=True)
    sand_state = sand_model.state()
    sand_state.body_q = wp.clone(robot.state.body_q)
    sand_state.body_qd = wp.zeros_like(robot.state.body_qd)
    sand_state.body_f = wp.zeros_like(robot.state.body_f)
    reaction = wp.zeros_like(robot.state.body_f)
    viewer = None
    try:
        if not args.headless:
            from mujoco import viewer as mj_viewer
            viewer = mj_viewer.launch_passive(host, data)
            # The host viewer has a bounded overlay budget; all particles still
            # participate in physics. Enlarge sampled dots for an illustrative view.
            if surface is None:
                stride = max(1, int(np.ceil(sand_model.particle_count / viewer.user_scn.maxgeom)))
                visible = np.arange(0, sand_model.particle_count, stride)
                dot_radius = float(min(spacing) * .5 * stride ** (1 / 3))
                print(f'Viewer draws {len(visible)} / {sand_model.particle_count} sand particles', flush=True)
            else:
                with viewer.lock():
                    surface.update(sand_state.particle_q.numpy())
                viewer.update_hfield(surface.hfield)
                viewer.sync()
                print('Viewer draws a continuous sand surface with fine grain texture', flush=True)
        print('Settling sand with robot held at its initial pose...', flush=True)
        for _ in range(round(cfg['settle_seconds'] / dt)):
            sand.step(sand_state, sand_state, control=None, contacts=None, dt=dt)
        print('Starting G1 pose PD and two-way sand coupling (no balance policy).', flush=True)
        max_reaction = 0.
        steps = int(np.ceil(args.seconds / dt))
        for frame in range(steps):
            if viewer is not None and not viewer.is_running():
                break
            start = time.perf_counter()
            for _ in range(cfg['rigid_substeps']):
                torque = kp * (target - data.qpos[qadr]) - kd * data.qvel[vadr]
                data.ctrl[motors] = np.clip(torque, -limits, limits)
                robot._push()
                wp.launch(add_reaction, dim=model.body_count, inputs=[reaction, robot.state.body_f])
                robot.solver.step(robot.state, robot.next_state, robot.control, None, host.opt.timestep)
                robot.state, robot.next_state = robot.next_state, robot.state
                robot._pull()
            wp.launch(prepare_colliders, dim=model.body_count,
                      inputs=[dt, robot.state.body_q, robot.state.body_qd, reaction,
                              model.body_inv_mass, model.body_inv_inertia,
                              sand_state.body_q, sand_state.body_qd])
            sand.step(sand_state, sand_state, control=None, contacts=None, dt=dt)
            impulses, positions, ids = sand.collect_collider_impulses(sand_state)
            reaction.zero_()
            wp.launch(sand_reaction, dim=ids.shape[0],
                      inputs=[dt, ids, impulses, positions, sand.collider_body_index,
                              robot.state.body_q, model.body_com, reaction])
            force = reaction.numpy()
            max_reaction = max(max_reaction, float(np.linalg.norm(force[:, :3], axis=1).max()))
            if not np.isfinite(data.qpos).all() or not np.isfinite(force).all():
                raise RuntimeError('non-finite robot state or sand reaction')
            if viewer is not None:
                positions_cpu = sand_state.particle_q.numpy()
                with viewer.lock():
                    if surface is not None:
                        surface.update(positions_cpu)
                    else:
                        viewer.user_scn.ngeom = len(visible)
                        for i, particle in enumerate(visible):
                            mujoco.mjv_initGeom(viewer.user_scn.geoms[i], mujoco.mjtGeom.mjGEOM_SPHERE,
                                np.full(3, dot_radius), positions_cpu[particle].astype(np.float64),
                                np.eye(3).ravel(), np.array([.76, .62, .36, 1.], dtype=np.float32))
                if surface is not None:
                    viewer.update_hfield(surface.hfield)
                viewer.sync()
                time.sleep(max(0., dt - (time.perf_counter() - start)))
            if frame % 100 == 0:
                print(f't={data.time:.2f}s pelvis={data.qpos[2]:.3f}m max sand force={max_reaction:.1f}N', flush=True)
            if data.qpos[2] < .35:
                print('Stopped: robot fell; pose PD does not provide balance on sand.')
                break
        particles = sand_state.particle_q.numpy()
        if not np.isfinite(particles).all():
            raise RuntimeError('non-finite sand particles')
        print(f'Finished: {data.time:.3f}s, max body sand force={max_reaction:.2f}N; '
              f'particle z={particles[:, 2].min():.3f}..{particles[:, 2].max():.3f}m')
    finally:
        if viewer is not None:
            viewer.close()


if __name__ == '__main__':
    main()
