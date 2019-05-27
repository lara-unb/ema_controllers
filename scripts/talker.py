#!/usr/bin/env python
# license removed for brevity
import rospy

# import ros msgs
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import Int8
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String

# import utilities
from math import pi
from tf import transformations

# global variables
global angle
global time

angle = [0,0]
time = [0,0]

def imu_callback(data):
    # get timestamp
    time.append(data.header.stamp)

    # get angle position
    qx,qy,qz,qw = data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w
    euler = transformations.euler_from_quaternion([qx, qy, qz, qw], axes='rzyx')

    x = euler[2]
    y = euler[1]

    # correct issues with more than one axis rotating
    if y >= 0:
        y = (y/pi) * 180
        if abs(x) > (pi*0.5):
            y = 180-y            
    else:
        y = (y/pi) * 180
        if abs(x) > (pi*0.5):
            y = 180 - y
        else:
            y = 360 + y

    angle.append(y)

    # get angular speed
    #speed.append(data.angular_velocity.y*(180/pi))

    # get error
    #speed_err.append(speed_ref - speed[-1])

    # print latest
    #print time[-1], angle[-1], speed[-1], speed_err[-1]


def talker():

	# subscribed topics
    sub = {}
    sub['pedal'] = rospy.Subscriber('imu/pedal', Imu, callback = imu_callback)

	# published topics
    pub = {}
    pub['talker']  = rospy.Publisher('chatter', String, queue_size=10)
    pub['angle'] = rospy.Publisher('control/angle', Float64, queue_size=10)

    # build basic angle message
    angleMsg = Float64()

	# init
    rospy.init_node('talker', anonymous=True)

    # define loop rate (in hz)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():

		# talker
        #hello_str = "hello!! %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub['talker'].publish(hello_str)

        # send angle update
        angleMsg.data = angle[-1]
        pub['angle'].publish(angleMsg)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
