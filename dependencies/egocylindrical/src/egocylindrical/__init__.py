from __future__ import print_function
from __future__ import division
from builtins import str
from builtins import object

from past.utils import old_div

import math
import numpy as np
from threading import RLock

## This code was supposedly originally in egocylindrical/src/egocylindrical/ec_camera_model.py,
## but I have found no trace of that file
class ECCameraModel(object):

    def __init__(self):
        self.mutex = RLock()

    def set_info(self, camera_info):
        dims = camera_info.points.layout.dim

        if len(dims)>=3:
            with self.mutex:
                self.height = dims[1].size
                self.width = dims[2].size
                if len(dims)>=4:
                    self.can_width = dims[3].size

                self.vfov = camera_info.fov_v
                self.hscale = old_div(self.width, (2 * math.pi))
                self.vscale = old_div(self.height, self.vfov)
        else:
            raise RuntimeError("Not enough dimensions defined in info message!")

    def project_pixels_to_3d_rays(self, uv):
        uv = np.asarray(uv)
        uv = uv.reshape(2,-1)
        ray = np.ndarray(shape=(3, uv.shape[1]), dtype=float)

        x = uv[0,:] #?
        y = uv[1,:] #?

        with self.mutex:
            theta = old_div((x - (old_div(self.width, 2))), self.hscale)
            ray[1, :] = old_div((y - (old_div(self.height, 2))), self.vscale)

        ray[0,:] = np.sin(theta);
        ray[2,:] = np.cos(theta);
        ##The above codes already ensures that x^2 + z^2 =1
        #ranges = ray[:,0] * ray[:,0] + ray[:,2] * ray[:,2]
        #ray /= ranges
        return ray

    def project_pixels_to_3d_points(self, uv, image):
        uv = np.asarray(uv)
        uv = uv.reshape(2, -1)
        ray = self.project_pixels_to_3d_rays(uv)

        coord = (uv[1,:],uv[0,:])
        depth = image[coord]
        points = ray * depth[:,None]
        return points

    def points_to_ranges(self, points):
        points = np.asarray(points)
        points = points.reshape(3, -1)
        ranges = np.sqrt(np.square(points[0,:])+np.square(points[2,:]))
        return ranges

    def points_to_x_idx(self, points):
        points = np.asarray(points)
        points = points.reshape(3, -1)
        with self.mutex:
            x_idx = np.arctan2(points[0,:], points[2,:])*self.hscale + old_div(self.width,2)
        return x_idx

    def points_to_y_idx(self, points):
        points = np.asarray(points)
        points = points.reshape(3, -1)
        ranges = self.points_to_ranges(points=points)
        proj_point_ys = np.divide(points[1,:], ranges)
        with self.mutex:
            y_idx =  proj_point_ys * self.vscale + old_div(self.height,2)
        return y_idx

    def project_3d_points_to_pixels(self, points):
        points = np.asarray(points)
        points = points.reshape(3, -1)
        with self.mutex:
            x_idx = self.points_to_x_idx(points=points)
            y_idx = self.points_to_y_idx(points=points)
        pixels = np.vstack((x_idx, y_idx))
        return pixels



def test_camera_model():
    model = ECCameraModel()
    model.height = 128
    model.width = 512
    model.vfov = old_div(math.pi,2)
    model.update()

    num_pts = 50

    pixels = np.random.randint(low=(0, 0), high = (model.height, model.width), size=(num_pts,2)).transpose()
    print("Pixels:\n" + str(pixels))

    proj_pnts = model.project_pixels_to_3d_rays(uv=pixels)
    print("Projected rays:\n" + str(proj_pnts))

    proj_pixels = model.project_3d_points_to_pixels(points = proj_pnts)
    print("Reprojected pixels:\n" + str(proj_pixels))
    #points = [[0.0, 0.0, 2.0], [0.5, 0.0, 1.8], [-0.5, 0.3, -1.8]]

    print((pixels == proj_pixels))

    diff = np.abs(pixels - proj_pixels)
    print("Diffs:\n" + str(diff))


