# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Usage: plot_scattered_path_out data.txt output_name')
        exit(1)
    else:
        path = sys.argv[1]
        output_name = sys.argv[2]
        gps_path = np.loadtxt(path)
        fig = plt.figure()
        plt.scatter(gps_path[:, 2], gps_path[:, 3], c=gps_path[:, gps_path.shape[1] - 1], cmap='autumn')
        plt.colorbar()
        plt.savefig(output_name + ".pdf", dpi=150)

        exit(1)
