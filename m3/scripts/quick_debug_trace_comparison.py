# coding=UTF-8
import sys
import numpy as np
import matplotlib

# matplotlib.use('Agg')
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D


def PlotPath(filepath, fix_path, origin_path, gps_path):
    plt.figure('Trajectories 2d')
    plt.title("Trajectories", fontsize=16)
    p0, = plt.plot(origin_path[:, 53], origin_path[:, 54], 'r')
    p1, = plt.plot(fix_path[:, 53], fix_path[:, 54], 'g')
    plt.legend([p0, p1], ["origin", "fix"]);
    plt.grid()
    plt.savefig(filepath + "/trajectory-2d.pdf", dpi=150)

    fig = plt.figure('Trajectories 3d')
    plt.title("Trajectories 3d", fontsize=16)
    ax = Axes3D(fig)
    p00, = ax.plot(origin_path[:, 53], origin_path[:, 54], origin_path[:, 55], 'r-')
    p01, = ax.plot(fix_path[:, 53], fix_path[:, 54], fix_path[:, 55], 'g-')
    ax.legend(['origin', 'fix'])
    plt.grid
    plt.savefig(filepath + "/trajectory-3d.pdf", dpi=150)

    fig = plt.figure('Gnss 3d')
    plt.title("Gnss 3d", fontsize=16)
    ax = Axes3D(fig)
    p00, = ax.plot(gps_path[:, 2], gps_path[:, 3], gps_path[:, 4], 'r-')
    p01, = ax.plot(fix_path[:, 53], fix_path[:, 54], fix_path[:, 55], 'g-')
    ax.legend(['gnss', 'fix'])
    plt.grid
    plt.savefig(filepath + "/gnss-3d.pdf", dpi=150)

    plt.figure('Gnss 2d')
    plt.title("Gnss 2d", fontsize=16)
    p0, = plt.plot(gps_path[:, 2], gps_path[:, 3], 'r.')  # r表示颜色，v表示下三角线类型,:点线
    p1, = plt.plot(fix_path[:, 53], fix_path[:, 54], 'g.')
    plt.legend([p0, p1], ["gps", "fix"])
    plt.grid()
    plt.savefig(filepath + "/gps-2d.pdf", dpi=150)

    plt.figure('Trajectory id')
    plt.title("Trajectory id", fontsize=16)
    plt.scatter(origin_path[:, 53], origin_path[:, 54], c=origin_path[:, 1], cmap='jet')
    plt.grid()
    plt.colorbar()
    plt.xlabel('X/m')
    plt.ylabel('Y/m')
    plt.savefig(filepath + "/trajectory-id.pdf", dpi=150)

    plt.show()


def LoadMappingTxt(filepath):
    fix_path = np.loadtxt(filepath + "/keyframes.txt")
    # origin_path = np.loadtxt(filepath + "/merge_keyframes.txt")
    origin_path = np.loadtxt(filepath + "/keyframes.txt")
    gps_path = np.loadtxt(filepath + "/gps_path.txt")

    PlotPath(filepath, fix_path, origin_path, gps_path)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input vaild param !!!')
        exit(1)
    else:
        path = sys.argv[1]
        LoadMappingTxt(path)
        exit(1)
