# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

x = []
y = []
z = []
last = 0
lastx = 0
lasty = 0
lastz = 0

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input vaild param !!!')
        exit(1)
    else:
        path = sys.argv[1]
        keyframes = np.loadtxt(path + 'keyframes.txt')
        fig = plt.figure('All path 2d')
        for a in keyframes:
            if a[1] == last:
                x.append(a[53])
                y.append(a[54])
                z.append(a[0])
            else:
                fig = plt.figure(str(last))
                plt.plot(x, y, "r.")
                for n, x, y in zip(z[:], x[:], y[:]):
                    plt.text(x+0.1, y+0.1, '%d' %n, {'color': 'y', 'fontsize': 10, 'ha': 'center', 'va': 'center'})
                fig = plt.figure('All path 2d')
                plt.plot(keyframes[:, 53], keyframes[:, 54], 'r*')
                plt.show()
                x = list()
                y = list()
                z = list()
                lastx = a[53]
                lasty = a[54]
                lastz = a[0]
                x.append(lastx)
                y.append(lasty)
                z.append(lastz)
                last = a[1]
        fig = plt.figure(str(22))
        plt.plot(x, y, "r.")
        for n, x, y in zip(z[:], x[:], y[:]):
            plt.text(x+0.1, y+0.1, '%d' %n, {'color': 'y', 'fontsize': 10, 'ha': 'center', 'va': 'center'})
        fig = plt.figure('All path 2d')
        plt.scatter(keyframes[:, 53], keyframes[:, 54], c=keyframes[:, 60], cmap='autumn')
        plt.colorbar()
        plt.show()

