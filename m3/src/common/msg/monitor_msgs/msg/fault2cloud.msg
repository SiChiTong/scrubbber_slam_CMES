monitor_msgs/fault_info[]  info_vec    ##故障
int32   fault_level        ##整车故障等级 0：不处理1：减速2：急停
int32   auto_drive		   ##自动驾驶能力
int32   parallel_drive	   ##平行驾驶能力
int32   romote_drive	   ##遥控驾驶能力
int32   vehicle_state	   ##车辆状态 0：初始化,1：休眠,2：遥控就绪,3：自动就绪,4：遥控驾驶,5：自动驾驶,6：人工干预,7：急停,8：故障
float64 velocity   		   ##整车速度(odom) ivmsglocpos.msg
float64 heading            ##相对真北朝向
float64 longitude          ##经度
float64 latitude           ##纬度
float64 globle_x           ##全局坐标系横轴
float64 globle_y           ##全局坐标系纵轴
int32   gps_status         ##GPS状态: 4为正常
int32   location_status    ##0:好1:一般2:不可用
float64 target_angle       ##目标转角 ivsteercontrol.msg
int32   steer_torque       ##横向驱动扭矩
float64 target_torque      ##驱动扭矩 ivdrivercontrol.msg
int32   actuator_mode      ##执行器模式:0-减速；1-加速;2-紧急制动
int32   shift_position     ##档位信息:0-无效;1-R档;2-N档;3-D档
int32   current_eps_state  ##EPS状态 ivactuator.msg
int32   uisteer_angle      ##实际转向角度 ivactuator.msg
float64 real_strtorque     ##实际转向力矩
float64 current_batsoc     ##电池组剩余电量 ivbattery.msg