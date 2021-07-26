# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def PlotPath(keyframes):
    fig = plt.figure('All path 2d')
    p10, = plt.plot(keyframes[:, 15], keyframes[:, 16], 'g-')
    p11, = plt.plot(keyframes[:, 3], keyframes[:, 4], 'y-')
    p12, = plt.plot(keyframes[:, 52], keyframes[:, 53], 'r-')
    for n, x, y in zip(keyframes[:,0], keyframes[:,3], keyframes[:,4]):
        plt.text(x+0.1, y+0.1, '%d' %n, {'color': 'b', 'fontsize': 10, 'ha': 'center', 'va': 'center'})
    
    fig = plt.figure('All path 3d')
    ax = Axes3D(fig)
    p00, = ax.plot(keyframes[:, 3], keyframes[:, 4], keyframes[:, 5], 'y-')
    p01, = ax.plot(keyframes[:, 15], keyframes[:, 16], keyframes[:, 17], 'g-')
    p03, = ax.plot(keyframes[:, 52], keyframes[:, 53], keyframes[:, 54], 'r-')
    plt.title("All path", fontsize=16)
    plt.grid()
    plt.show()


# def PlotPath(filepath, keyframes, optimized_path, beratio, opt_beratio):
#     fig = plt.figure('All path 2d')
#     p10, = plt.plot(keyframes[:, 15], keyframes[:, 16], 'g-')
#     p11, = plt.plot(keyframes[:, 3], keyframes[:, 4], 'y-')
#     p12, = plt.plot(keyframes[:, 52], keyframes[:, 53], 'r-')
#     for n, x, y in zip(keyframes[:,0], keyframes[:,52], keyframes[:,53]):
#         plt.text(x+0.1, y+0.1, '%d' %n, {'color': 'b', 'fontsize': 10, 'ha': 'center', 'va': 'center'})
    
#     fig = plt.figure('All path 3d')
#     ax = Axes3D(fig)
#     p00, = ax.plot(keyframes[:, 3], keyframes[:, 4], keyframes[:, 5], 'y-')
#     p01, = ax.plot(keyframes[:, 15], keyframes[:, 16], keyframes[:, 17], 'g-')
#     p02, = ax.plot(optimized_path[:, 4], optimized_path[:, 5], optimized_path[:, 6], 'c-')
#     p03, = ax.plot(keyframes[:, 52], keyframes[:, 53], keyframes[:, 54], 'r-')
#     plt.title("All path", fontsize=16)
#     plt.grid()

    # fig = plt.figure('Layer')
    # Floor1 = fig.add_subplot(221)
    # p31 = plt.scatter(optimized_path[:, 4], optimized_path[:, 5], 12, opt_beratio[:, 4], 'o')
    # plt.colorbar(p31)
    # Floor2 = fig.add_subplot(222)
    # p32 = plt.scatter(optimized_path[:, 4], optimized_path[:, 5], 12, opt_beratio[:, 2], 'o')
    # plt.colorbar(p32)
    # BE_RATIO = fig.add_subplot(223)
    # p33, = plt.plot(beratio[:,0], beratio[:, 2], 'b-')
    # p34, = plt.plot(beratio[:,0], beratio[:, 3], 'r-')
    # BE_RATIO = fig.add_subplot(224)
    # p35, = plt.plot(opt_beratio[:,0], opt_beratio[:, 2], 'b-')
    # p36, = plt.plot(opt_beratio[:,0], opt_beratio[:, 3], 'r-')
    
    # plt.show()


def LoadMappingTxt(filepath):
    keyframes = np.loadtxt(filepath + 'keyframes.txt')
    # optimized_path = np.loadtxt(filepath + 'optimized_path.txt')
    # beratio = np.loadtxt(filepath + 'beratio.txt')
    # opt_beratio = np.loadtxt(filepath + 'opt_beratio.txt')
    # opt_beratio = ''
    # all_connection = np.loadtxt(filepath + 'all_connection_type.txt')
    # all_floor = np.loadtxt(filepath + 'all_layer_type.txt')
    
    # PlotPath(filepath,keyframes, optimized_path, beratio, opt_beratio)
    PlotPath(keyframes)

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input vaild param !!!')
        exit(1)
    else:
        path = sys.argv[1]
        LoadMappingTxt(path)
        exit(1)
        exit(1)
