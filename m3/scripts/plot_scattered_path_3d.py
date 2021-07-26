# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input vaild param !!!')
        exit(1)
    else:
        path = sys.argv[1]
        path_data = np.loadtxt(path)
        fig = plt.figure()
        ax = Axes3D(fig)
        ax.scatter(path_data[:,1], path_data[:,2], path_data[:,3], c=path_data[:,path_data.shape[1]-1], cmap='jet')
        plt.show()

        exit(1)
