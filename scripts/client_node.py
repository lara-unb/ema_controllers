#!/usr/bin/env python
import rospy
import dynamic_reconfigure.client
from std_msgs.msg import String


def client_callback(config):
    rospy.loginfo("Config set to {enable_control}, {control_sel}, {enable_quad},{enable_hams},{current_quad},"
                  "{current_hams},{step_time}, {ref_path}, {Kp}, {Ki}, {Kd}, {ES_A}, {ES_omega}, {ES_phase}, "
                  "{ILC_alpha}, {ILC_beta}, {ILC_gama}, {co_activation}".format(**config))


if __name__ == "__main__":
    # init node
    rospy.init_node("client_node", anonymous=False)

    # # init client
    client1 = dynamic_reconfigure.client.Client("server", timeout=30, config_callback=client_callback)

    # init gui
    current_0 = 0
    enable_0 = True  # type: bool
    str_0 = "/home/bb8/git/ema_controllers/scripts/ema/dataset/reference_for_thesis.mat"
    # str_1 = "/home/bb8/Desktop/subject01_"
    str_1 = "/home/bb8/Documents/subject01_RIGHT_"
    # str_1 = "/home/bb8/Documents/subject01_LEFT_"
    dt = 4
    Kp_init = 0.00145
    Ki_init = 0.00061
    Kd_init = 0.00013
    ES_A_init = 800
    ES_omega_init = 8
    ES_phase_init = 0
    ILC_alpha_init = 0.2
    ILC_beta_init = 0.8
    ILC_gama_init = 0.02
    min_pw_initQ = 0
    min_pw_initH = 0

    client1.update_configuration(
        {"control_sel": 0, "channels_sel": 0, "current_quad": current_0, "current_hams": current_0, "enable_control": False,
         "enable_quad": enable_0, "enable_hams": enable_0, "ref_path": str_0, "step_time": dt, "Kp": Kp_init,
         "Ki": Ki_init, "Kd": Kd_init, "ES_A": ES_A_init, "ES_omega": ES_omega_init, "ES_phase": ES_phase_init,
         "ILC_alpha": ILC_alpha_init, "ILC_beta": ILC_beta_init, "ILC_gama": ILC_gama_init, "co_activation": False,
         "save_path": str_1, "min_pw_q": min_pw_initQ, "min_pw_h": min_pw_initH})

    # rate
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        rate.sleep()
