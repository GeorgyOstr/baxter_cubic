#!/usr/bin/python

import rospy
import baxter_interface
from constant_parameters import right_prepare_position, joint_states, Joint_Names, new_joint_states

#Recognition

#Solution

#Manipulation

def main():
    rospy.init_node("rsm_kubic")
    limb_r = baxter_interface.Limb("right")
    limb_l = baxter_interface.Limb("left")
    r = rospy.Rate(0.2)
    
    for i in new_joint_states:
        right_move = {'right_s0':i[11], 'right_s1':i[12], 'right_e0':i[9], 'right_e1':i[10],
                                                  'right_w0':i[13], 'right_w1':i[14], 'right_w2':i[15]}
        left_move = {'left_s0':i[4], 'left_s1':i[5], 'left_e0':i[2], 'left_e1':i[3],
                                              'left_w0':i[6], 'left_w1':i[7], 'left_w2':i[8]}
        
        limb_r.move_to_joint_positions(right_move)
        limb_l.move_to_joint_positions(left_move)
        r.sleep

    
if __name__ == "__main__":
    main()
