# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('usage: compare_path.py path1.txt path2.txt')
        exit(1)
    else:
        path1 = sys.argv[1]
        path2 = sys.argv[2]
        path1_data = np.loadtxt(path1)
        path2_data = np.loadtxt(path2)

        fig = plt.figure()
        ax = Axes3D(fig)
        ax.scatter(path1_data[:, 1], path1_data[:, 2], path1_data[:, 3], 'b')
        ax.scatter(path2_data[:, 1], path2_data[:, 2], path2_data[:, 3], 'r')

        delta = path1_data[:, 1:3] - path2_data[:, 1:3]
        fig2 = plt.figure()
        plt.plot(np.arange(delta.shape[0]), delta[:, 0])
        plt.show()

        exit(1)
