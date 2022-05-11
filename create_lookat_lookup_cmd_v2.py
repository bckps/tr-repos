from scipy.spatial.transform import Rotation as R
import numpy as np


class RotVector:
    def __init__(self):
        self.lookat = np.array([0, 0, -1])
        self.lookup = np.array([0, 1, 0])

    def convert_blender2simu(self, blen_vec):
        """
        座標の変換 Blender→bunnykiller
        
        Parameters
        ----------
        blen_vec : np.array([x,y,z])
            Blenderのベクトル

        Returns
        -------
        simu_vec : np.array([x,y,z])
            bunnykillerのベクトル
        """

        simu_vec = np.array([blen_vec[0],blen_vec[2], -blen_vec[1]])
        return simu_vec

    def apply_coordinates_LookatLookup(self, locvec, rotvec):
        """
        座標の変換を含めたカメラの位置と向きを計算
        
        Parameters
        ----------
        locvec : np.array([x,y,z])
            Blenderの位置ベクトル
        rotvec : np.array([rx,ry,rz])
            Blenderの回転ベクトル(degree)

        Returns
        -------
        lookat : np.array([x,y,z])
            カメラが見つめる方向
        lookup : np.array([x,y,z])
            カメラの上方向
        
        """
        
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


def test_func():
    rotv = RotVector()
    origin = np.array(rotv.convert_blender2simu([0.6488918, 0.6742092, 0.8883101]))#blender vec
    dist = np.array(rotv.convert_blender2simu([-6.05, -1.9246, 0.33]))
    orient = orient_vec(origin, dist)
    print(orient)

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