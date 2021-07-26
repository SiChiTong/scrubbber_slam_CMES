# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input vaild param !!!')
        exit(1)
    else:
        path = sys.argv[1]
        error_data = np.loadtxt(path)
        fig = plt.figure()
        plt.hist(error_data, bins=200,range=(0,8), cumulative=False, normed=True)
        plt.xlabel('RTK chi2 error')
        plt.ylabel('histogram %')
        plt.grid()
        plt.title('RTK error histogram')
        plt.show()

        exit(1)
