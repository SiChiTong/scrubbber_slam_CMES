import sys
import os
import time
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import multiprocessing


def plot_data_fit(x, y, title, xlabel, ylabel):
    plt.figure()
    plt.scatter(x, y)
    coef = np.polyfit(x, y, 1)
    coef_pretty = np.poly1d(coef)
    print(title + " poly coeffs:")
    print(coef_pretty)
    x_sort = np.sort(x)
    y_fit = np.polyval(coef, x_sort)
    plt.plot(x_sort, y_fit)
    plt.title(title)
    plt.grid()
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.savefig(title + ".png")


def test_pipeline(map_data):
    print ('testing map ' + map_data[0] + ' with link ' + map_data[1])
    os.system('bin/pipeline_test --map_name=' + map_data[0] + ' --map_link=' + map_data[1] + " 1>/dev/null 2>&1")
    # clean original data
    print ('rm -rf /home/idriver/data/' + map_data[0] + "/*")
    os.system('rm -rf /home/idriver/data/' + map_data[0] + "/*")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print ('Usage: python run_full_test.py benchmark.txt')
        exit(1)
    if os.path.exists('./bin/mapping') == False:
        print ('Please run this script at mapping 3.0 root path!')
        exit(1)

    f = open(sys.argv[1])
    test_dataset = []

    time_usage = []
    bag_size = []
    map_area = []
    success = []
    simulation_status = []

    # workers = multiprocessing.cpu_count()
    workers = 4
    pool = multiprocessing.Pool(processes=workers)
    print('testing with workers = %d' % (workers))

    for line in f.readlines():
        if line[0] == '#':
            continue
        [index, name, url] = line.split()
        test_dataset.append((name, url))

    # clean disk
    # os.system('rm -rf /home/idriver/data/*')
    # os.system('rm -rf /home/idriver/results/*')

    print ('total tested datasets: %d' %(len(test_dataset)))
    pool.map(test_pipeline, test_dataset)

    # get results from task info
    for dataset_name in test_dataset:
        result_file = open('/home/idriver/results/' + dataset_name[0] + "/task_info.txt")
        first_line = result_file.readline().split()
        time_usage.append(float(first_line[1]) / (60.0 * 60.0))
        bag_size.append(float(first_line[3]) / (1024.0 * 1024.0 * 1024.0))
        map_area.append(float(first_line[5]) / (10000.0))
        success.append(int(first_line[6]))
        simulation_status.append(int(first_line[8]))

    n_tested = len(success)
    print('success rate: %f %%' % (sum(success) / n_tested * 100.0))
    s0 = simulation_status.count(0)
    s1 = simulation_status.count(1)
    s2 = simulation_status.count(2)
    print('simulation PASS/CHECK/FAIL: %d/%d/%d' % (s0, s1, s2))
    print('simulation PASS/CHECK/FAIL rates: %f%%/%f%%/%f%%' % (
        s0 / n_tested * 100, s1 / n_tested * 100, s2 / n_tested * 100))

    # bag size vs. time usage
    plot_data_fit(bag_size, time_usage, "size_time", "GB", "Hours")
    # map area vs. time usage
    plot_data_fit(map_area, time_usage, "area_time", "10k m2", "Hours")
