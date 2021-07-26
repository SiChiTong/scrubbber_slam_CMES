# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt


def PlotPath( matching_path, degeneracy_path):
    fig = plt.figure('degeneracy 2d')
    
    matching = fig.add_subplot(121)
    p1, = plt.plot(matching_path[:, 4], matching_path[:, 5], 'k-')

    degeneracy = fig.add_subplot(122)
    p3 = plt.scatter(matching_path[:, 4], matching_path[:, 5], 12, degeneracy_path[:, 2], 'o')
    plt.colorbar(p3)

    plt.title("degeneracy", fontsize=16)
    plt.grid()

    plt.show()


def LoadMappingTxt(filepath):
    matching_path = np.loadtxt(filepath + "matching_path.txt")
    degeneracy_path = np.loadtxt(filepath + "matching_degeneracy.txt")
    PlotPath( matching_path, degeneracy_path)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input vaild param !!!')
        exit(1)
    else:
        path = sys.argv[1]
        LoadMappingTxt(path)
        exit(1)
        exit(1)
