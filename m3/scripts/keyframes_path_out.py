# coding=UTF-8
import sys
import numpy as np
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def PlotPath(orig_path, optimized_path2):
    plt.figure('Process path')
    plt.scatter(optimized_path2[:, 2], optimized_path2[:, 3], c=optimized_path2[:, 9], cmap='jet')
    plt.text(optimized_path2[0, 2], optimized_path2[0, 3], 'optimized path', {'color': 'r', 'fontsize': 12}, alpha=0.5)

    plt.title("Trajectories", fontsize=16)
    plt.grid()
    plt.colorbar()
    plt.savefig(orig_path + "/trajectory.pdf", dpi=150)


def LoadMappingTxt(filepath):
    optimized_path2 = np.loadtxt(filepath + "/optimization2.txt")
    PlotPath(filepath, optimized_path2)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input vaild param !!!')
        exit(1)
    else:
        path = sys.argv[1]
        LoadMappingTxt(path)
        exit(1)
