# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def PlotPath(before_opt_keyframes, keyframes):
    # fig = plt.figure('2d')
    # before_connected2d = fig.add_subplot(121)
    # p10, = plt.plot(before_opt_keyframes[:, 15], before_opt_keyframes[:, 16], 'g-')
    # p11, = plt.plot(before_opt_keyframes[:, 3], before_opt_keyframes[:, 4], 'y-')
    # connected2d = fig.add_subplot(122)
    # p20, = plt.plot(keyframes[:, 15], keyframes[:, 16], 'g-')
    # p21, = plt.plot(keyframes[:, 3], keyframes[:, 4], 'y-')
 
    fig2 = plt.figure('before_connected3d')
    ax = Axes3D(fig2)
    p100, = ax.plot(before_opt_keyframes[:, 3], before_opt_keyframes[:, 4], before_opt_keyframes[:, 5], 'y-')
    p101, = ax.plot(before_opt_keyframes[:, 15], before_opt_keyframes[:, 16], before_opt_keyframes[:, 17], 'g-')
   
    fig3 = plt.figure('connected3d')
    ax = Axes3D(fig3)
    p200, = ax.plot(keyframes[:, 3], keyframes[:, 4], keyframes[:, 5], 'y-')
    p201, = ax.plot(keyframes[:, 15], keyframes[:, 16], keyframes[:, 17], 'g-')
    
    plt.show()


def LoadMappingTxt(filepath):
    before_opt_keyframes = np.loadtxt(filepath + 'bef_opt_keyframes.txt');
    keyframes = np.loadtxt(filepath + 'keyframes.txt')

    PlotPath(before_opt_keyframes, keyframes)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input vaild param !!!')
        exit(1)
    else:
        path = sys.argv[1]
        LoadMappingTxt(path)
        exit(1)
        exit(1)
