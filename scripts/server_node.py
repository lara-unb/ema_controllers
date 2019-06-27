#!/usr/bin/env python
import rospy

from dynamic_reconfigure.server import Server
from ema_controllers.cfg import TutorialsConfig
from ema_controllers.msg import Gait_GUI

# global guiMsg
#
# guiMsg = Gait_GUI()


# call back
def callback(config, level):
	rospy.loginfo("""Reconfigure Request: {enable_control}, {control_sel},{enable_quad},{enable_hams}, {current_quad},
	{current_hams},{step_time},{ref_path},{Kp},{Ki},{Kd}""".format(**config))

	if config['enable_control']:
		config.groups.groups.Ref_param.state = False
		config.groups.groups.Enable_config.state = False
	else:
		config.groups.groups.Ref_param.state = True
		config.groups.groups.Enable_config.state = True

	if config['control_sel'] == 0:
		config.groups.groups.PID_param.state = False
		config.groups.groups.ESC_param.state = False
		config.groups.groups.ILC_param.state = False

	elif config['control_sel'] == 1:
		config.groups.groups.PID_param.state = True
		config.groups.groups.ESC_param.state = False
		config.groups.groups.ILC_param.state = False

	elif config['control_sel'] == 2:
		config.groups.groups.PID_param.state = True
		config.groups.groups.ILC_param.state = True
		config.groups.groups.ESC_param.state = False

	elif config['control_sel'] == 3:
		config.groups.groups.PID_param.state = True
		config.groups.groups.ILC_param.state = False
		config.groups.groups.ESC_param.state = True

	elif config['control_sel'] == 4:
		config.groups.groups.PID_param.state = False
		config.groups.groups.ILC_param.state = False
		config.groups.groups.ESC_param.state = False

	else:
		config.groups.groups.PID_param.state = False
		config.groups.groups.ILC_param.state = False
		config.groups.groups.ESC_param.state = False

	return config


if __name__ == "__main__":
	rospy.init_node("server", anonymous = False)  # init node
	srv = Server(TutorialsConfig, callback)  # call server
	rospy.spin()
