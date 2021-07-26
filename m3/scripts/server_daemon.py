#!/usr/bin/env python
# coding: utf-8
import os
import sys
import time
import subprocess

while True:
    out = subprocess.Popen('ps -aux | grep \"mapping server\"',shell=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,close_fds=True)
    res = out.stdout.readlines()
    mapping_active = False
    for r in res:
        if r.find(b'grep') == -1 and r.find(b'mapping server') != -1:
            mapping_active = True
    print (mapping_active)

    if mapping_active == False:
        if os.path.exists('./bin/mapping'): 
            os.system('bin/mapping server > /dev/null 2>&1 &')  # launch a new server
            print ('bin/mapping server > /dev/null 2>&1 &')
        elif os.path.exists('./mapping'):
            os.system('./mapping server > /dev/null 2>&1 &')  # launch a new server
            print ('./mapping server > /dev/null 2>&1 &')
        else:
            print 'dont know where to find the mapping executable'
    time.sleep(120)
