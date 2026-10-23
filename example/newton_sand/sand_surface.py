"""Render MPM particles as a smooth, visual-only height field."""
import mujoco
import numpy as np
from scipy.ndimage import distance_transform_edt, gaussian_filter


class SandSurface:
    def __init__(self, model, lower, upper, particle_spacing):
        self.model = model
        self.hfield = model.hfield('sand_surface').id
        self.rows = int(model.hfield_nrow[self.hfield])
        self.cols = int(model.hfield_ncol[self.hfield])
        self.base = float(lower[2]) + .001
        self.scale = float(model.hfield_size[self.hfield, 2])
        self.center = (lower[:2] + upper[:2]) * .5
        self.half_size = model.hfield_size[self.hfield, :2].copy()
        self.origin = self.center - self.half_size
        self.cell = 2 * self.half_size / [self.cols - 1, self.rows - 1]
        self.radius = float(min(particle_spacing) * .5)
        self.support = float(max(particle_spacing) * 1.5)
        self.edge_width = float(max(particle_spacing))
        self.sigma = max(.7, float(min(particle_spacing) / min(self.cell) * .4))

    def update(self, particles):
        points = np.asarray(particles)
        indices = np.rint((points[:, :2] - self.origin) / self.cell).astype(int)
        valid = (np.isfinite(points).all(axis=1) & (indices[:, 0] >= 0) &
                 (indices[:, 0] < self.cols) & (indices[:, 1] >= 0) & (indices[:, 1] < self.rows))
        heights = np.full((self.rows, self.cols), -np.inf)
        ix, iy = indices[valid].T
        np.maximum.at(heights, (iy, ix), points[valid, 2] + self.radius)
        occupied = np.isfinite(heights)
        if occupied.any():
            # Extend only locally: empty parts of the bed must not grow a sand sheet.
            distance, nearest = distance_transform_edt(
                ~occupied, sampling=self.cell[::-1], return_indices=True)
            nearest_height = heights[tuple(nearest)]
            # Taper the outer edge to the substrate, then remove particle-scale bumps.
            weight = np.clip((self.support + self.edge_width - distance) / self.edge_width, 0., 1.)
            heights = self.base + (np.maximum(nearest_height, self.base) - self.base) * weight
            heights = gaussian_filter(heights, self.sigma, mode='constant', cval=self.base)
        else:
            heights.fill(self.base)
        start = int(self.model.hfield_adr[self.hfield])
        self.model.hfield_data[start:start + heights.size] = np.clip(
            (heights.ravel() - self.base) / self.scale, 0., 1.)


def create_surface_scene(scene, lower, upper, particle_spacing):
    """Add the display surface to the host scene; Newton imports the original scene."""
    spec = mujoco.MjSpec.from_file(str(scene))
    # A one-centimeter display grid is independent of the coarser MPM physics grid.
    half_size = (upper[:2] - lower[:2]) * .5 + .15
    cols, rows = np.ceil(half_size * 2 / .01).astype(int) + 1
    spec.add_hfield(name='sand_surface', nrow=int(rows), ncol=int(cols),
                    size=[*half_size, .6, .005], userdata=np.zeros(int(rows * cols)))
    rng = np.random.default_rng(7)
    grain = np.clip(rng.normal(1., .055, (256, 256, 1)), .8, 1.2)
    pixels = np.clip(grain * [194, 163, 112], 0, 255).astype(np.uint8)
    spec.add_texture(name='fine_sand', type=mujoco.mjtTexture.mjTEXTURE_2D,
                     width=256, height=256, nchannel=3, data=pixels.ravel())
    textures = [''] * int(mujoco.mjtTextureRole.mjNTEXROLE)
    textures[int(mujoco.mjtTextureRole.mjTEXROLE_RGB)] = 'fine_sand'
    spec.add_material(name='fine_sand', textures=textures, texrepeat=[6, 6],
                      texuniform=True, specular=.08, shininess=.03, reflectance=0.,
                      rgba=[1, 1, 1, 1])
    center = (lower[:2] + upper[:2]) * .5
    spec.worldbody.add_geom(name='sand_surface_visual', type=mujoco.mjtGeom.mjGEOM_HFIELD,
                            hfieldname='sand_surface', pos=[*center, float(lower[2]) + .001],
                            material='fine_sand', contype=0, conaffinity=0, group=1)
    model = spec.compile()
    return model, SandSurface(model, lower, upper, particle_spacing)
