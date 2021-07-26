# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt

last = 0
lastx = 0
lasty = 0
x = []
y = []

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Usage: plot_scattered_path_out data.txt output_name')
        exit(1)
    else:
        path = sys.argv[1]
        test_info = np.loadtxt(path + "splite/splite_map_info_test.txt")
        kf_info = np.loadtxt(path + "keyframes.txt")
        for a in test_info:
            if a[0] == last:
                x.append(a[1])
                y.append(a[2])
            else :
                fig = plt.figure()
                plt.plot(x, y, 'r.')
                x = []
                y = []
                x.append(a[1])
                y.append(a[2])
            last = a[0]
            lastx = a[1]
            lasty = a[2]
        x.append(lastx)
        y.append(lasty)
        fig = plt.figure()
        plt.plot(x, y, 'r.')
        fig1 = plt.figure("trj")
        plt.scatter(kf_info[:, 53], kf_info[:, 54], c=kf_info[:, 1] * 20, cmap='autumn')
        plt.colorbar()
        fig2 = plt.figure("result")
        plt.scatter(test_info[:, 1], test_info[:, 2], c=test_info[:, 0] * 50, cmap='autumn')
        plt.colorbar()
        plt.show()
        plt.close()
        exit(1)

