"""Physics engines behind the existing MJCF/DDS observation interface.

MuJoCo model/data remain the host-side scene, sensor and viewer interface.
Only the selected engine advances time. Newton imports the same MJCF and uses
SolverMuJoCo (MuJoCo Warp on CUDA, its reference CPU solver on CPU).
"""
from pathlib import Path
from importlib.metadata import version

import mujoco
import numpy as np


class MujocoBackend:
    name = 'mujoco'

    def __init__(self, model, data):
        self.model, self.data = model, data

    def step(self):
        mujoco.mj_step(self.model, self.data)

    def prepare(self):
        pass


class NewtonBackend:
    name = 'newton'

    def __init__(self, model, data, scene, device='cuda:0'):
        self.model, self.data = model, data
        self.scene, self.device = Path(scene).resolve(), device
        self.solver = None

    def _initialize(self):
        # Lazy import keeps the default MuJoCo installation independent of Newton.
        try:
            import newton
            import warp as wp
            from newton.solvers import SolverMuJoCo
        except ImportError as error:
            raise RuntimeError('Newton 1.7.0 dependencies are missing; run: '
                               'python -m pip install -r simulate_python/requirements-newton.txt') from error
        if version('newton') != '1.7.0':
            raise RuntimeError('This backend requires newton==1.7.0; run: '
                               'python -m pip install -r simulate_python/requirements-newton.txt')
        self.wp, self.newton = wp, newton
        wp.init()
        builder = newton.ModelBuilder()
        builder.add_mjcf(str(self.scene), ctrl_direct=True, collapse_fixed_joints=False)
        self.joints = []
        self.bodies = []
        for ni, label in enumerate(builder.body_label):
            mi = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, label.rsplit('/', 1)[-1])
            if mi < 0:
                raise ValueError(f'Newton body missing from host scene: {label}')
            self.bodies.append((ni, mi))
        mapped = set()
        for ni, kind in enumerate(builder.joint_type):
            if kind == newton.JointType.FIXED:
                continue
            child = builder.joint_child[ni]
            if kind == newton.JointType.FREE:
                body = dict(self.bodies)[child]
                matches = np.flatnonzero((self.model.jnt_bodyid == body) &
                                        (self.model.jnt_type == mujoco.mjtJoint.mjJNT_FREE))
                if len(matches) != 1:
                    raise ValueError('expected one matching free joint')
                mi = int(matches[0])
            elif kind in (newton.JointType.REVOLUTE, newton.JointType.PRISMATIC):
                name = builder.joint_label[ni].rsplit('/', 1)[-1]
                mi = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                if mi < 0:
                    raise ValueError(f'Newton joint missing from host scene: {name}')
            else:
                raise ValueError(f'unsupported joint type: {kind}')
            nq, nv = builder.joint_q_start[ni], builder.joint_qd_start[ni]
            mq, mv = self.model.jnt_qposadr[mi], self.model.jnt_dofadr[mi]
            self.joints.append((kind, nq, nv, mq, mv, child))
            mapped.add(mi)
            if kind != newton.JointType.FREE:
                builder.joint_armature[nv] = float(self.model.dof_armature[mv])
                builder.joint_friction[nv] = float(self.model.dof_frictionloss[mv])
                builder.joint_damping[nv] = float(self.model.dof_damping[mv])
        if len(mapped) != self.model.njnt:
            raise ValueError('Newton import did not preserve all scene joints')
        self.engine_model = builder.finalize(device=self.device)
        self.state = self.engine_model.state()
        self.next_state = self.engine_model.state()
        self.control = self.engine_model.control()
        if self.control.mujoco.ctrl.size != self.model.nu:
            raise ValueError('Newton import did not preserve the motor interface')
        self.com = self.engine_model.body_com.numpy()
        parent_frames = self.engine_model.joint_X_p.numpy()
        child_frames = self.engine_model.joint_X_c.numpy()
        self.free_frames = {}
        for ni, kind in enumerate(builder.joint_type):
            if kind == newton.JointType.FREE:
                parent = self._transform(parent_frames[ni])
                child = self._transform(child_frames[ni])
                # Newton 1.7 expresses free-joint pose and twist in the parent anchor frame.
                quat = parent_frames[ni][[6, 3, 4, 5]]
                self.free_frames[builder.joint_q_start[ni]] = (parent, child, self._rotation(quat))
        self.solver = SolverMuJoCo(self.engine_model, use_mujoco_cpu=self.device == 'cpu',
                                   use_mujoco_contacts=True)

    def prepare(self):
        if self.solver is None:
            self._initialize()

    @staticmethod
    def _rotation(quat):
        matrix = np.empty(9)
        mujoco.mju_quat2Mat(matrix, np.asarray(quat, dtype=np.float64))
        return matrix.reshape(3, 3)

    def _transform(self, pose):
        return self.wp.transform(self.wp.vec3(*pose[:3]), self.wp.quat(*pose[3:7]))

    def _push(self):
        q, v = self.state.joint_q.numpy(), self.state.joint_qd.numpy()
        for kind, nq, nv, mq, mv, child in self.joints:
            if kind == self.newton.JointType.FREE:
                pos = self.data.qpos[mq:mq+7]
                rotation = self._rotation(pos[3:7])
                omega = rotation @ self.data.qvel[mv+3:mv+6]
                parent, child_frame, parent_rotation = self.free_frames[nq]
                world = self._transform(np.r_[pos[:3], pos[4:7], pos[3]])
                relative = self.wp.transform_multiply(self.wp.transform_inverse(parent), world)
                q[nq:nq+7] = np.asarray(self.wp.transform_multiply(relative, child_frame))
                velocity = self.data.qvel[mv:mv+3] + np.cross(omega, rotation @ self.com[child])
                v[nv:nv+6] = np.r_[parent_rotation.T @ velocity, parent_rotation.T @ omega]
            else:
                q[nq], v[nv] = self.data.qpos[mq], self.data.qvel[mv]
        self.state.joint_q.assign(q)
        self.state.joint_qd.assign(v)
        self.newton.eval_fk(self.engine_model, self.state.joint_q, self.state.joint_qd, self.state)
        forces = np.zeros((self.engine_model.body_count, 6), dtype=np.float32)
        for ni, mi in self.bodies:
            forces[ni] = self.data.xfrc_applied[mi]
        self.state.body_f.assign(forces)
        self.control.mujoco.ctrl.assign(self.data.ctrl.astype(np.float32))

    def step(self):
        self.prepare()
        self._push()  # Also honors viewer resets and fixed-base diagnostic edits.
        self.solver.step(self.state, self.next_state, self.control, None, self.model.opt.timestep)
        self.state, self.next_state = self.next_state, self.state
        self._pull()

    def _pull(self):
        """Update the host scene after a Newton step (also used by coupled examples)."""
        q, v = self.state.joint_q.numpy(), self.state.joint_qd.numpy()
        for kind, nq, nv, mq, mv, child in self.joints:
            if kind == self.newton.JointType.FREE:
                parent, child_frame, parent_rotation = self.free_frames[nq]
                world = self.wp.transform_multiply(parent, self._transform(q[nq:nq+7]))
                world = np.asarray(self.wp.transform_multiply(world, self.wp.transform_inverse(child_frame)))
                quat = world[[6, 3, 4, 5]]
                rotation = self._rotation(quat)
                omega = parent_rotation @ v[nv+3:nv+6]
                self.data.qpos[mq:mq+7] = np.r_[world[:3], quat]
                self.data.qvel[mv:mv+6] = np.r_[parent_rotation @ v[nv:nv+3] -
                    np.cross(omega, rotation @ self.com[child]), rotation.T @ omega]
            else:
                self.data.qpos[mq], self.data.qvel[mv] = q[nq], v[nv]
        self.data.time += self.model.opt.timestep
        # Refresh existing sensors and ray casting without integrating host dynamics.
        mujoco.mj_forward(self.model, self.data)


def create_backend(name, model, data, scene, device='cuda:0'):
    if name == 'mujoco':
        return MujocoBackend(model, data)
    if name == 'newton':
        return NewtonBackend(model, data, scene, device)
    raise ValueError(f'unknown physics backend: {name}')
