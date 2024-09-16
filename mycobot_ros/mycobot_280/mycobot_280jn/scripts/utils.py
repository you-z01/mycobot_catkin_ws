
from pymycobot import *

mc = MyCobot("/dev/ttyTHS1", 1000000)
str = ""
def instruct_control(instruct):
	# [0, 0, 0, 0, 0, 0, 0， 8]
	# 0 和 8 作校验位
	
	if instruct[0] == 1 and instruct[6] == 1:
		str = "机械臂放电"
		mc.power_off()	# 机械臂放电
		return str
	elif instruct[0] == 2 and instruct[6] == 2:
		str = "机械臂上电"
		mc.power_on() 	# 机械臂上电
		return str
	elif instruct[0] == 3 and instruct[6] == 3:
		str = "获取到机械臂位姿:"
		coor = mc.get_coords()
		str = str + str(coor)
		return str
	elif instruct[0] == 4 and instruct[6] == 4:
		str = "获取到机械臂角度:"
		ang = mc.get_angles()
		str = str + str(ang)
		return str

	else:
		return

	# if list[0] == 0 and list[7] == 9:
	# 	list_send = list[1:6]
	# 	return True, list_send
	# else:
	# 	return False, None

