#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import scipy.io as sio
import dynamic_reconfigure.client as reconfig


def gui_callback(config):
    global control_onoff
    global reference_path
    global step_time

    if config['enable_control']:
        control_onoff = True
    else:
        control_onoff = False

    reference_path = config['ref_path']
    step_time = config['step_time']


def get_reference():
    global control_onoff
    global reference_path
    global step_time

    rospy.init_node('reference_knee', anonymous=True)

    # published topics
    pub = {}
    pub['ref'] = rospy.Publisher('ref_channel', Float64, queue_size=10)
    pub['steps'] = rospy.Publisher('steps_channel', Float64, queue_size=10)

    # communicate with the dynamic server
    dyn_params = reconfig.Client('server', config_callback=gui_callback)

    step_time = 3  # se eu alterar aqui eu preciso alterar na GUI!
    # gait01 e gait03: 751 dados
    # gait04: 1000 dados
    # ref..angle, refnewangle: 10000
    # reference_for_thesis

    # 1500 dados em x segundos:
    new_freq = 1500/step_time

    rate = rospy.Rate(new_freq)

    control_onoff = False
    count = 0
    number_steps = 0
    # step_time = 0.5

    while not rospy.is_shutdown():

        if control_onoff:
            # load reference
            try:
                reference_data = sio.loadmat(reference_path)
                knee_ref = reference_data['knee_ref'][0]
            except:
                print("Ooops, the file you tried to load is not in the folder.")
                knee_ref = [0, 0, 0, 0]

            if count > len(knee_ref) - 1:
                count = 0
                number_steps = number_steps + 1

            # check step_time for new frequency
            # if step_time == 0:
            #     print("Ooops, step time cannot be zero, the frequency was not changed.")
            # else:
            #     new_freq = len(knee_ref) / step_time

            # send messages
            pub['ref'].publish(knee_ref[count])
            pub['steps'].publish(number_steps)

            # update counter
            count = count + 1

        else:
            count = 0
            number_steps = 0
            # new_freq = 2
            pub['ref'].publish(0)
            pub['steps'].publish(number_steps)

        # rate = rospy.Rate(new_freq)
        rate.sleep()


if __name__ == '__main__':
    try:
        get_reference()
    except rospy.ROSInterruptException:
        pass
