#
# Created by pengguoqi on 19-7-24.
#

import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

def ShowMatchData(filepath):
    fig = plt.figure('match_data')
    pose_filter = np.loadtxt(filepath + "match_data.txt")
    trans = fig.add_subplot(221)
    p7, = plt.plot(pose_filter[:, 0], 'r+')
    p8, = plt.plot(pose_filter[:, 1], 'k-')
    plt.legend([p7, p8], ["trans", "scor"])
    lamnda5 = fig.add_subplot(222)
    p6, = plt.plot(pose_filter[:,2], 'b-')
    plt.legend([p6],["lambda5"])
    lamnda0 = fig.add_subplot(223)
    p1, = plt.plot(pose_filter[:,3], 'r-')
    plt.legend([p1],["lambda0"])
    # loc = fig.add_subplot(224)
    # p2, = plt.plot(pose_filter[:,5], pose_filter[:,6])
    # plt.legend([p2],["loc"])
    loc = fig.add_subplot(224)
    p2, = plt.plot(pose_filter[:,4], 'r-')
    plt.legend([p2],["lambda5/lambda0"])

    plt.grid()
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) != 2 :
        print 'Please input vaild param !!!'
        exit(1)
    path = sys.argv[1]
    ShowMatchData(path)
    exit(1)
