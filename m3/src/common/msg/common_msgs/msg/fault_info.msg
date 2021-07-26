float64 timestamp_sec   ##故障发生时间
string  module_name		##故障模块或节点名称
int32   sequence_num	##消息数
int32   version			##版本号
int32   error_code		##故障码
int32   fault_level		##故障等级
uint8   fault_type		##0:硬件故障;1:软件故障;2:算法故障;3:其他故障;4:外设故障;5:电路故障;6:电源故障;7:通讯故障
uint8   notify_type		##0:不处理;  1:APP消息提示；2:APP屏保提示；（所有等级的故障都可以列表查看）
string  msg				##故障描述