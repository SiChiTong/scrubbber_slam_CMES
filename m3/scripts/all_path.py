# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def PlotPath(keyframes, vertices):
    fig = plt.figure('All path 3d')
    ax = Axes3D(fig)

    # matching pose
    p00, = ax.plot(keyframes[:, 7], keyframes[:, 8], keyframes[:, 9], 'y-')
    # dr pose
    p01, = ax.plot(keyframes[:, 20], keyframes[:, 21], keyframes[:, 22], 'g-')

    # gps pose
    # gps_path = np.asarray([line for line in keyframes if np.fabs(line[33]) > 1e-6 and np.fabs(line[34]) > 1e-6],
    #                       dtype=np.float32)
    # if gps_path.shape[0] != 0:
    #     p02, = ax.plot(gps_path[:, 33], gps_path[:, 34], gps_path[:, 35], 'b-')

    # optimized pose 1
    # p03, = ax.plot(keyframes[:, 46], keyframes[:, 47], keyframes[:, 48], 'm-')

    # stage 2
    p04, = ax.plot(keyframes[:, 53], keyframes[:, 54], keyframes[:, 55], 'r-')

    # p04, = ax.plot(vertices[:, 1], vertices[:, 2], vertices[:,3], 'm-')

    ax.legend(['Matching', 'DR', 'optimized'])
    plt.title("All path", fontsize=16)

    fig = plt.figure('All path 2d')

    # matching
    p0, = plt.plot(keyframes[:, 7], keyframes[:, 8], 'y-')

    # dr
    # p1, = plt.plot(keyframes[:, 20], keyframes[:, 21], 'g-')

    # gps
    # p2, = plt.plot(gps_path[:, 33], gps_path[:, 34],  'b-')

    # opti stage 1
    # p3, = plt.plot(keyframes[:, 46], keyframes[:, 47],  'm-')

    # opti stage 2
    p4, = plt.plot(keyframes[:, 53], keyframes[:, 54], 'r-')

    # plt.scatter(gps_path[:, 33], gps_path[:, 34], c=gps_path[:, 3], cmap='jet')

    plt.grid()
    # plt.colorbar()
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Matching vs Optimized')
    # plt.legend(['RTK', 'optimized', 'RTK status'])
    plt.legend(['Matching', 'optimized'])
    plt.show()


def LoadMappingTxt(filepath):
    keyframes = np.loadtxt(filepath + 'keyframes.txt')
    # vertices = np.loadtxt(filepath + 'vertices.txt')
    vertices = ''

    PlotPath(keyframes, vertices)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input vaild param !!!')
        exit(1)
    else:
        path = sys.argv[1]
        LoadMappingTxt(path)
        exit(1)
        exit(1)
