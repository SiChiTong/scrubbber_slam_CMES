#
# Created by pengguoqi on 19-7-24.
#

import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def SavePathResult(filepath, flag):
    fig = plt.figure('trans')
    pose_filter = np.loadtxt(filepath + "path.txt")
    # path
    fig1 = plt.figure('path')
    plt.title("Path", fontsize=16)
    p7, = plt.plot(pose_filter[:, 4], pose_filter[:, 5], 'r+')
    p8, = plt.plot(pose_filter[:, 7], pose_filter[:, 8], 'k-')
    p9, = plt.plot(pose_filter[:, 10], pose_filter[:, 11], 'g-')
    plt.legend([p7, p8, p9], ["ndt", "gps", "msf"]);
    path_pdf = filepath + "path.pdf"
    path_svg = filepath + "path.svg"
    plt.savefig(path_pdf)
    # plt.savefig(path_svg)
    # ndt_theshold
    fig2 = plt.figure('ndt_threshold')
    plt.title("NDT threshold", fontsize=16)
    p6, = plt.plot(pose_filter[:, 3], label=' ')
    plt.legend([p6], ["ndt_threshold"]);
    path_pdf = filepath + "ndtthreshold.pdf"
    path_svg = filepath + "ndtthreshold.svg"
    plt.savefig(path_pdf)
    # plt.savefig(path_svg)
    # trans_probability
    fig3 = plt.figure('trans_probability')
    plt.title("Trans Probability", fontsize=16)
    sc = plt.scatter(pose_filter[:, 4], pose_filter[:, 5], 12, pose_filter[:, 14], '+')
    plt.colorbar(sc)
    path_pdf = filepath + "transprobability.pdf"
    path_svg = filepath + "transprobability.svg"
    plt.savefig(path_pdf)
    # plt.savefig(path_svg)
    # loc_angle
    fig4 = plt.figure('locangle')
    plt.title("Location Angle", fontsize=16)
    p10, = plt.plot(pose_filter[:, 13], 'r-')
    plt.legend([p10], ["loc_angle"]);
    path_pdf = filepath + "locangle.pdf"
    path_svg = filepath + "locangle.svg"
    plt.savefig(path_pdf)
    # plt.savefig(path_svg)
    # gps
    fig5 = plt.figure('gps_status')
    plt.title("GPS Status", fontsize=16)
    gpsv = plt.scatter(pose_filter[:, 10], pose_filter[:, 11], 12, pose_filter[:, 21], '+')
    plt.colorbar(gpsv)
    path_pdf = filepath + "gpsstatus.pdf"
    path_svg = filepath + "gpsstatus.svg"
    plt.savefig(path_pdf)
    # plt.savefig(path_svg)
    # gps and loc distance
    fig6 = plt.figure('distance')
    plt.title("GPS and Location Distance", fontsize=16)
    gpslocv = plt.scatter(pose_filter[:, 10], pose_filter[:, 11], 12, pose_filter[:, 22], '+')
    plt.colorbar(gpslocv)
    path_pdf = filepath + "gpslocdistance.pdf"
    path_svg = filepath + "gpslocdistance.svg"
    plt.savefig(path_pdf)
    # plt.savefig(path_svg)
    if flag == "0" :
        fig7 = plt.figure('trajectory status')
        plt.title("Trajectories Status", fontsize=16)
        tralocs = plt.scatter(pose_filter[:, 10], pose_filter[:, 11], 12, pose_filter[:, 23], '+')
        plt.colorbar(tralocs)
        path_pdf = filepath + "trajectories_status.pdf"
        path_svg = filepath + "trajectories_status.svg"
        plt.savefig(path_pdf)
        # plt.savefig(path_svg)


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print ('Please input vaild param !!!')
        exit(1)
    path = sys.argv[1]
    flag = sys.argv[2]
    SavePathResult(path,flag)
    exit(1)
