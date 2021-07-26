#!/usr/bin/python
import sys 
import math
import matplotlib.pyplot as plt
import rosbag
from mpl_toolkits.mplot3d import Axes3D
from pyproj import Proj
import numpy as np

# reload(sys)
# sys.setdefaultencoding('utf-8')
def getY(q):
    w =q[3]
    x =q[0]
    y =q[1]
    z =q[2]
    xx = 1-2*(z*z+y*y)
    if xx == 0:
        return 0
    angle = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
    return angle*180/3.1415926

def HeadingToAngle(q):
    yaw = 90 - q
    if yaw > 180:
        yaw = yaw - 360
    if yaw < -180:
        yaw = yaw + 360
    return yaw

tn=[]
fusion_utc=[]
fusion_lon = []
fusion_lat = []
fusion_yaw = []
dr_time_n = []
fusion_v = []
fusion_roll = []
fusion_pitch = []

tgps = []
gps_lon = []
gps_lat = []
gps_yaw = []
gps_heading_valid = []
gps_status = []
gps_utctime = []
gps_x = []
gps_y = []

fusion_x = []
fusion_y = [] 

first_utm_x = -1
first_utc_time = -1

offset_x = 666793.64833850693
offset_y = 3523440.4433874395
ant_x = 0.61
ant_y = -0.5

if len(sys.argv) < 4:
    print('please input command: python xx.py xx.bag opt***_path.txt gps***_path.txt')
bags=[]
txts = []
bags.append(sys.argv[1])
if len(sys.argv)>=3:
  txts.append(sys.argv[2])
if len(sys.argv)>=4:
  txts.append(sys.argv[3])

p = Proj(proj='utm',zone=50,ellps='WGS84', preserve_units=False)

for name in bags:
        bag = rosbag.Bag(name)
        for topic, msg, t in bag.read_messages(topics=['/tpimu']):
            if msg.llh.lon.variable > 30 and msg.llh.lat.variable > 30:
                tn.append(t.to_sec())
                fusion_lon.append(msg.llh.lon.variable)
                fusion_lat.append(msg.llh.lat.variable)
                if first_utc_time < 0:
                    first_utc_time = msg.dr_pose_and_time.utctime
                fusion_utc.append(msg.dr_pose_and_time.utctime - first_utc_time)
                yaw_fusion = msg.pose.pose_euler.rotation.yaw.variable
                if yaw_fusion > 180:
                    yaw_fusion = yaw_fusion - 360
                fusion_yaw.append(yaw_fusion)

                fusion_roll.append(msg.pose.pose_euler.rotation.roll.variable)
                fusion_pitch.append(msg.pose.pose_euler.rotation.pitch.variable)

                dr_time_n.append(msg.dr_pose_and_time.dr_time)
                if first_utm_x < 0:
                    if abs(msg.pose.pose_euler.position.x.variable - offset_x) < 2000 and abs(msg.pose.pose_euler.position.y.variable - offset_y) < 2000:
                        first_utm_x = offset_x
                        first_utm_y = offset_y
                    else:
                        first_utm_x = msg.pose.pose_euler.position.x.variable
                        first_utm_y = msg.pose.pose_euler.position.y.variable
                fusion_x.append(msg.pose.pose_euler.position.x.variable - first_utm_x)
                fusion_y.append(msg.pose.pose_euler.position.y.variable - first_utm_y)
                fusion_v.append(msg.v.variable)
        for topic, msg, t in bag.read_messages(topics=['/ivsensorgps']):
            if msg.utctime - first_utc_time > -10 and msg.lon > 30  and msg.lat > 30:
                tgps.append(t.to_sec())
                gps_lon.append(msg.lon)
                gps_lat.append(msg.lat)
                gps_yaw.append(msg.heading)
                gps_utctime.append(msg.utctime - first_utc_time)
                gps_heading_valid.append(msg.is_heading_valid)
                gps_status.append(msg.status)
                
        bag.close()


if len(txts) > 0:
    opt_x = []
    opt_y = []
    optimize_path = txts[0]
    opt_x = [float(l.split()[4]) for l in open(optimize_path)]
    opt_y = [float(l.split()[5]) for l in open(optimize_path)]

if len(txts) > 1:
    gps_x_opt = []
    gps_y_opt = []
    gps_path = txts[1]
    gps_x_opt = [float(l.split()[4]) for l in open(gps_path)]
    gps_y_opt = [float(l.split()[5]) for l in open(gps_path)]

gps_xx,gps_yy = p(gps_lon, gps_lat)
gps_x = [x-first_utm_x for x in gps_xx]
gps_y = [y-first_utm_y for y in gps_yy]

gps_x_ant = []
gps_y_ant = []
x_count = 0
for x in gps_x:
    yaw = gps_yaw[x_count]
    # yaw = 90 - yaw
    if yaw > 180.0:
        yaw = yaw - 360.0
    if yaw < -180.0:
        yaw = yaw + 360.0
    y = gps_y[x_count]
    y = y - ant_y * math.sin(yaw * 3.14159/180.0) + ant_x * math.cos(yaw * 3.14159/180.0)
    x = x - (-1.0 * ant_y) * math.cos(yaw * 3.14159/180.0) + ant_x * math.sin(yaw * 3.14159/180.0)
    gps_x_ant.append(x)
    gps_y_ant.append(y)
    x_count = x_count + 1  


plt.figure('fusion-gps_ori_x-y')
plt.plot(fusion_x,fusion_y,'r*-', gps_x, gps_y, 'y+-', gps_x_ant, gps_y_ant, 'b+-')

plt.figure('fusion-optimize_x-y')
plt.plot(fusion_x,fusion_y,'r*-', opt_x, opt_y, 'y+-', gps_x_opt, gps_y_opt, 'b+-', gps_x_ant, gps_y_ant, 'g+-')

plt.show()
