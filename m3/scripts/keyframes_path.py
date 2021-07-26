# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def PlotPath(dr_path, gps_path, matching_path, optimized_path, optimized_path2, merge_path, degeneracy_path):
    fig = plt.figure('All path 3d')
    ax = Axes3D(fig)

    # p0, = ax.plot(dr_path[:, 1], dr_path[:, 2], dr_path[:, 3], 'g-')
    p1, = ax.plot(gps_path[:, 2], gps_path[:, 3], gps_path[:, 4], 'b-')
    # p2, = ax.plot(matching_path[:, 1], matching_path[:, 2], matching_path[:, 3], 'y-')
    p3, = ax.plot(optimized_path[:, 2], optimized_path[:, 3], optimized_path[:, 4], 'c-')
    p4, = ax.plot(optimized_path2[:, 2], optimized_path2[:, 3], optimized_path2[:, 4], 'r-')

    plt.title("All path", fontsize=16)
    plt.grid()

    fig = plt.figure('All path 2d')

    # p0, = plt.plot(dr_path[:, 1], dr_path[:, 2], 'g-')
    p1, = plt.plot(gps_path[:, 2], gps_path[:, 3], 'b-')
    # p2, = plt.plot(matching_path[:, 1], matching_path[:, 2], 'y-')
    p3, = plt.plot(optimized_path[:, 2], optimized_path[:, 3], 'c-')
    p4, = plt.plot(optimized_path2[:, 2], optimized_path2[:, 3], 'r-')

    plt.show()


def LoadMappingTxt(filepath):
    dr_path = np.loadtxt(filepath + 'dr_path.txt')
    gps_path = np.loadtxt(filepath + 'gps_path.txt')
    # remove zero entry
    gps_path = np.asarray([line for line in gps_path if np.fabs(line[1]) > 1e-6 and np.fabs(line[2]) > 1e-6],
                          dtype=np.float32)

    matching_path = np.loadtxt(filepath + "matching_path.txt")
    optimized_path = np.loadtxt(filepath + "optimization1.txt")
    optimized_path2 = np.loadtxt(filepath + "optimization2.txt")
    merge_path = ''
    degeneracy_path = ''
    PlotPath(dr_path, gps_path, matching_path, optimized_path, optimized_path2, merge_path, degeneracy_path)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input vaild param !!!')
        exit(1)
    else:
        path = sys.argv[1]
        LoadMappingTxt(path)
        exit(1)
        exit(1)
