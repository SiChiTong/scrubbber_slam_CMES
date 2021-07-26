#
# Created by pengguoqi on 19-7-24.
#

import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

def SavePathResult(filepath):
    fig = plt.figure('trans')
    pose_filter = np.loadtxt(filepath + "path.txt")
    # path
    path = fig.add_subplot(221)
    p7, = plt.plot(pose_filter[:, 4], pose_filter[:, 5], 'r+')
    p8, = plt.plot(pose_filter[:, 7], pose_filter[:, 8], 'k-')
    p9, = plt.plot(pose_filter[:, 10], pose_filter[:, 11], 'g-')
    plt.legend([p7, p8, p9], ["ndt", "gps", "msf"]);
    deltatime = fig.add_subplot(222)
    p10, = plt.plot(pose_filter[:, 24], 'k-')
    plt.legend([p10], ["deltatime"])
    gps_value_status = fig.add_subplot(223)
    p6, = plt.plot(pose_filter[:,21], 'b-')
    plt.legend([p6],["gps_value_status"])
    dis = fig.add_subplot(224)
    p1, = plt.plot(pose_filter[:,22], 'r-')
    plt.legend([p1],["distance"])
    plt.show()
    plt.close()

if __name__ == '__main__':
    if len(sys.argv) != 2 :
        print 'Please input vaild param !!!'
        exit(1)
    path = sys.argv[1]
    SavePathResult(path)
    exit(1)
