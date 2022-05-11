from scipy.spatial.transform import Rotation as R
import numpy as np
np.set_printoptions(suppress=True)

class RotVector:
    def __init__(self):
        self.lookat = np.array([0, 0, -1])
        self.lookup = np.array([0, 1, 0])

    def convert_blender2simu(self, blen_vec):
        simu_vec = np.array([blen_vec[0],blen_vec[2], -blen_vec[1]])
        return simu_vec

    def perspectiveLookatLookup(self, locvec, rotvec):
        r = R.from_rotvec(rotvec, degrees=True)
        lookat = self.convert_blender2simu(r.apply(self.lookat))
        lookup = self.convert_blender2simu(r.apply(self.lookup))

        return lookat, lookup

    def blender_apply_coordinates_LookatLookup(self, locvec, rotvec):
        rx = R.from_rotvec(np.array([rotvec[0], 0, 0]).astype(float), degrees=True)
        ry = R.from_rotvec(np.array([0, rotvec[1], 0]).astype(float), degrees=True)
        rz = R.from_rotvec(np.array([0, 0, rotvec[2]]).astype(float), degrees=True)
        rot = rz * ry * rx
        lookat = rot.apply(self.lookat) + locvec
        lookup = rot.apply(self.lookup) + locvec

        return lookat, lookup

    def apply_coordinates_LookatLookup(self, locvec, rotvec):
        rx = R.from_rotvec(np.array([rotvec[0], 0, 0]).astype(float), degrees=True)
        ry = R.from_rotvec(np.array([0, rotvec[1], 0]).astype(float), degrees=True)
        rz = R.from_rotvec(np.array([0, 0, rotvec[2]]).astype(float), degrees=True)
        rot = rz * ry * rx
        scale = 1000
        lookat = scale * self.convert_blender2simu(rot.apply(self.lookat)) + self.convert_blender2simu(locvec)
        lookup = scale * self.convert_blender2simu(rot.apply(self.lookup)) + self.convert_blender2simu(locvec)
        # lookat = self.convert_blender2simu(rot.apply(self.lookat))
        # lookup = self.convert_blender2simu(rot.apply(self.lookup))
        return lookat, lookup

def orient_vec(origin, dist):
    v = dist - origin
    orinent = v / np.sqrt(np.sum(v ** 2))
    return orinent

def test1():
    origin = np.array([0.7988916, 0.2083092, 1.765791])#simu vec
    dist = np.array([-14.2, 11.1, -30])
    orient = orient_vec(origin, dist)
    print(orient)

    # print(37 * orient + origin)
    rotv = RotVector()
    rx = R.from_rotvec(np.array([107, 0, 0]).astype(float), degrees=True)
    ry = R.from_rotvec(np.array([0, 0, 0]).astype(float), degrees=True)
    rz = R.from_rotvec(np.array([0, 0, 25.2]).astype(float), degrees=True)
    rot = rz * ry * rx
    lookat = np.array([0, 0, -1])
    lookup = np.array([0, 1, 0])
    print(rotv.convert_blender2simu(rot.apply(lookat)))

def test2():
    rotv = RotVector()
    origin = np.array(rotv.convert_blender2simu([0.6488918, 0.6742092, 0.8883101]))#blender vec
    dist = np.array(rotv.convert_blender2simu([-6.05, -1.9246, 0.33]))
    orient = orient_vec(origin, dist)
    print(orient)

    # print(37 * orient + origin)
    rotv = RotVector()
    rx = R.from_rotvec(np.array([85.6, 0, 0]).astype(float), degrees=True)
    ry = R.from_rotvec(np.array([0, 0, 0]).astype(float), degrees=True)
    rz = R.from_rotvec(np.array([0, 0, 111]).astype(float), degrees=True)
    rot = rz * ry * rx
    lookat = np.array([0, 0, -1])
    lookup = np.array([0, 1, 0])
    print(rot.apply(lookat))

def test1_func():
    rotv = RotVector()
    origin = rotv.convert_blender2simu(np.array([0.7988916, -1.765791, 0.2083092]))#simu vec
    dist = rotv.convert_blender2simu(np.array([-14.2, 30, 11.1]))
    orient = orient_vec(origin, dist)
    print(orient)

    # print(37 * orient + origin)
    rotv = RotVector()
    lookat, lookup = rotv.apply_coordinates_LookatLookup(np.array([0, 0, 0]).astype(float),
                                                         np.array([107, 0, 25.2]).astype(float))
    print(lookat)

def test2_func():
    rotv = RotVector()
    origin = np.array(rotv.convert_blender2simu([0.6488918, 0.6742092, 0.8883101]))#blender vec
    dist = np.array(rotv.convert_blender2simu([-6.05, -1.9246, 0.33]))
    orient = orient_vec(origin, dist)
    print(orient)

    # print(37 * orient + origin)
    rotv = RotVector()
    lookat, lookup = rotv.apply_coordinates_LookatLookup(np.array([0.6488918, 0.6742092, 0.8883101]).astype(float),
                                                         np.array([85.6, 0, 111]).astype(float))
    print(origin)
    print(lookat)
    print(lookup)

if __name__ =='__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('x', type=float, help='positioin x')
    parser.add_argument('y', type=float, help='positioin y')
    parser.add_argument('z', type=float, help='positioin z')
    parser.add_argument('rx', type=float, help='rotation rx')
    parser.add_argument('ry', type=float, help='rotation ry')
    parser.add_argument('rz', type=float, help='rotation rz')
    args = parser.parse_args()
    rotv = RotVector()
    cam_loc = rotv.convert_blender2simu(np.array([args.x, args.y, args.z]).astype(float))
    lookat, lookup = rotv.apply_coordinates_LookatLookup(np.array([args.x, args.y, args.z]).astype(float),
                                                         np.array([args.rx, args.ry, args.rz]).astype(float))
    print('Location: ', cam_loc)
    print('Look up: ', lookup)
    print('Look at: ', lookat)