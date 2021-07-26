# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

def GetYaw(x, y, z, w):
    xx = 1-2*(z*z+y*y)
    if xx == 0:
        return 0
    angle = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
    return angle*180/3.1415926

def PlotPath(dr_path, gps_path, matching_path, optimized_path, optimized_path2, merge_path, degeneracy_path):
    fig = plt.figure('All path 3d')
    ax = Axes3D(fig)

    p0, = ax.plot(dr_path[:, 2], dr_path[:, 3], dr_path[:, 4], 'g-')

    p1, = ax.plot(gps_path[:, 2], gps_path[:, 3], gps_path[:, 4], 'b-')

    p2, = ax.plot(matching_path[:, 2], matching_path[:, 3], matching_path[:, 4], 'y-')

    p3, = ax.plot(optimized_path[:, 2], optimized_path[:, 3], optimized_path[:, 4], 'c-')

    p4, = ax.plot(optimized_path2[:, 2], optimized_path2[:, 3], optimized_path2[:, 4], 'r-')
  
    plt.title("All path", fontsize=16)
    plt.grid()


    fig = plt.figure('All path 2d')

    p0, = plt.plot(dr_path[:, 2], dr_path[:, 3], 'g-')

    p1, = plt.plot(gps_path[:, 2], gps_path[:, 3],  'b-')

    p2, = plt.plot(matching_path[:, 2], matching_path[:, 3],  'y-')

    p3, = plt.plot(optimized_path[:, 2], optimized_path[:, 3], 'c-')

    p4, = plt.plot(optimized_path2[:, 2], optimized_path2[:, 3], 'r-')

    heading_gps = []
    heading_gps = list(map(lambda x: GetYaw(x[0], x[1], x[2], x[3]), zip(gps_path[:, 5], gps_path[:, 6], gps_path[:, 7], gps_path[:, 8])))

    heading_opt = []
    heading_opt = list(map(lambda x: GetYaw(x[0], x[1], x[2], x[3]), zip(optimized_path2[:, 5], optimized_path2[:, 6], optimized_path2[:, 7], optimized_path2[:, 8])))

    fig = plt.figure('gps and opt headeing')
    pheading_gps = plt.plot(gps_path[:, 0], heading_gps, 'b-')
    pheading_opt = plt.plot(optimized_path2[:, 0], heading_opt, 'r-')

    error_count=[]
    x_error=[]
    y_error=[]
    heading_error = []
    gps_count = 0
    opt_count = 0

    gps_large_x = []
    gps_large_y = []
    opt_large_x = []
    opt_large_y = []
    for gps_index in gps_path[:, 0]:
        opt_count_start = opt_count
        for opt_index in optimized_path2[:, 0][opt_count_start: ]:
            # if gps_index == opt_index and gps_path[:, 9][gps_count] == 1:
            if gps_index == opt_index:
                x_error_num = gps_path[:, 2][gps_count] - optimized_path2[:, 2][opt_count]
                y_error_num = gps_path[:, 3][gps_count] - optimized_path2[:, 3][opt_count]
                x_error.append(x_error_num)
                y_error.append(y_error_num)
                heading_error_value = heading_gps[gps_count] - heading_opt[opt_count]
                if heading_error_value < -180.0:
                    heading_error_value = heading_error_value + 360.0
                if heading_error_value > 180.0:
                    heading_error_value = heading_error_value - 360.0
                heading_error.append(heading_error_value)
                error_count.append(gps_count)
                if x_error_num * x_error_num > 0.01 :
                    gps_large_x.append(gps_path[:, 2][gps_count])
                    gps_large_y.append(gps_path[:, 3][gps_count])
                    opt_large_x.append(optimized_path2[:, 2][opt_count])
                    opt_large_y.append(optimized_path2[:, 3][opt_count])
                break

                opt_count = opt_count + 1
            opt_count = opt_count + 1
        gps_count = gps_count + 1
    
        fig = plt.figure('All path 2d')

    fig = plt.figure('path 2d: gps / opt')
    p1r, = plt.plot(gps_path[:, 2], gps_path[:, 3],  'b-')
    p4r, = plt.plot(optimized_path2[:, 2], optimized_path2[:, 3], 'r-')
    # p1r, = plt.plot(gps_large_x, gps_large_y,  'r-')
    plt.scatter(gps_large_x, gps_large_y,s=40, c="#000000", marker='p')
    plt.scatter(opt_large_x, opt_large_y,s=20, c="#000000", marker='o')



    fig = plt.figure('gps and opt x error')
    px_error = plt.plot(error_count, x_error, 'b-')

    fig = plt.figure('gps and opt y error')
    py_error = plt.plot(error_count, y_error, 'b-')

    fig = plt.figure('gps and opt heading error')
    pheading_error = plt.plot(error_count, heading_error, 'b-')

    average_heading_error = np.mean(heading_error)
    print("heading error average: %f" %average_heading_error)

    plt.show()


def LoadMappingTxt(filepath):
    dr_path = np.loadtxt(filepath + 'dr_path.txt')
    gps_path = np.loadtxt(filepath + 'gps_path.txt')
    # remove zero entry
    gps_path = np.asarray([line for line in gps_path if np.fabs(line[1]) > 1e-6 and np.fabs(line[2]) > 1e-6],
                          dtype=np.float32)

    matching_path = np.loadtxt(filepath + "matching_path.txt")
    optimized_path = np.loadtxt(filepath + "optimization1.txt")
    optimized_path2 = np.loadtxt(filepath + "optimization2.txt")
    # merge_path = np.loadtxt(filepath + "merge_path.txt")
    merge_path = ''
    degeneracy_path = ''
    PlotPath(dr_path, gps_path, matching_path, optimized_path, optimized_path2, merge_path, degeneracy_path)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input vaild param !!!')
        exit(1)
    else:
        path = sys.argv[1]
        LoadMappingTxt(path)
        exit(1)
        exit(1)
