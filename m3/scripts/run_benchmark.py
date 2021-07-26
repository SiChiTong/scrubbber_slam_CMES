import sys
import os
import time

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print 'Usage: python run_benchmark.py benchmark.txt'
        exit(1)
    if os.path.exists('./bin/mapping') == False:
        print 'Please run this script at mapping 3.0 root path!'
        exit(1)
    # os.system('python scripts/server_daemon.py &!')
    print 'start server and running benchmark, please wait'
    f = open(sys.argv[1])
    time.sleep(3)
    for line in f.readlines():
        if line[0] == '#':
            continue
        [name, url] = line.split()
        print 'bin/mapping start ' + name + ' ' + url
        os.system('bin/mapping start ' + name + ' ' + url)
