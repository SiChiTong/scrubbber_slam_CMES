#
# Created by pengguoqi on 19-7-24.
#

import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

def SavePathResult(filepath):
    fig = plt.figure('trans')
    dr_pose_filter = np.loadtxt(filepath + "simulation_dr_path.txt")
    # path
    path = fig.add_subplot(211)
    p0, = plt.plot(dr_pose_filter[:, 0], 'k.')
    plt.legend([p0], ["dr_time"])
    dr_pose = fig.add_subplot(212)
    p1, = plt.plot(dr_pose_filter[:, 1], dr_pose_filter[:, 2], 'r.')
    plt.legend([p1], ["dr_path"])
    plt.show()
    plt.close()

if __name__ == '__main__':
    if len(sys.argv) != 2 :
        print 'Please input vaild param !!!'
        exit(1)
    path = sys.argv[1]
    SavePathResult(path)
    exit(1)
