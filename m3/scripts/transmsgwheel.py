#!/usr/bin/python
import sys 
import math
import matplotlib.pyplot as plt
import rosbag
from avos_x.msg import ivsensorgps, ivsensorimu
from mainstream_msgs.msg import ivmsglocalization
from ivactuator.msg import ivwheelspeed
from velodyne_msgs.msg import VelodyneScan

def TransImu(imu):
    rimu = ivsensorimu()
    rimu.header = imu.header
    rimu.fault_vec = imu.fault_vec
    # rimu.update = True
    rimu.TimeTag = imu.TimeTag
    rimu.utcTime = imu.utcTime
    rimu.gyro_x = imu.gyro_x
    rimu.gyro_y = imu.gyro_y
    rimu.gyro_z = imu.gyro_z
    rimu.acce_x = imu.acce_x
    rimu.acce_y = imu.acce_y
    rimu.acce_z = imu.acce_z
    rimu.Temperature = imu.Temperature
    return rimu
def TransGps(gps):
    rgps = ivsensorgps()
    rgps.header = gps.header
    rgps.fault_vec = gps.fault_vec
    # rgps.update = True
    rgps.lon = gps.lon
    rgps.lat = gps.lat
    rgps.height = gps.height
    rgps.heading = gps.heading
    rgps.velocity = gps.velocity
    rgps.track_angle = gps.track_angle
    rgps.utctime = gps.utctime
    rgps.hdop = gps.hdop
    rgps.diff_age = gps.diff_age
    rgps.base_length = gps.base_length
    rgps.heading_std = gps.heading_std
    rgps.status = gps.status
    rgps.satenum = gps.satenum
    rgps.status_yaw = gps.status_yaw
    rgps.is_heading_valid = gps.is_heading_valid
    return rgps
def TransWheel(wheel):
    rwheel=ivwheelspeed()
    #rwheel = wheel.wheel_speed
    rwheel.header = wheel.wheel_speed.header
    rwheel.wheelspeed_lr_pluse = wheel.wheel_speed.wheelspeed_lr_pluse
    rwheel.wheelspeed_rr_pluse = wheel.wheel_speed.wheelspeed_rr_pluse
    # rwheel.wheelspeed_lr = wheel.wheel_speed.wheelspeed_lr
    # rwheel.wheelspeed_rr = wheel.wheel_speed.wheelspeed_rr
    rwheel.pluse_mask = wheel.wheel_speed.pluse_mask
    return rwheel

if __name__ == '__main__':
    reload(sys)
    sys.setdefaultencoding('utf-8')
    if len(sys.argv)!=3:
        print '*****************************************************'
        print 'argv error. please: python *.py input.bag output.bag...'
        print '*****************************************************'
        sys.exit()
    name_in = sys.argv[1]
    name_out = sys.argv[2]
    rebag = rosbag.Bag(name_out, 'w')
    bag = rosbag.Bag(name_in)
    for topic, msg, t in bag.read_messages():
          if topic.endswith('ivsensorimu'):
              rebag.write('/ivsensorimu', TransImu(msg), t)
          elif topic.endswith('ivsensorgps'):
              rebag.write('/ivsensorgps', TransGps(msg), t)
          elif topic.endswith('velodyne_packets_1'):
              rebag.write('/velodyne_packets_1',msg,t)
          elif topic.endswith('tpcansensor'):
              rebag.write('/ivwheelspeed',TransWheel(msg),t)
        # elif topic.endswith('tplocalization'):
        #     rebag.write('/tplocalization',msg,t)
        # elif topic.endswith('ndt_status'):
        #     rebag.write('/ndt_status',msg,t)
    bag.close()
    rebag.close()
      
