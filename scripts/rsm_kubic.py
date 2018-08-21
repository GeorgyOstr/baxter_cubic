#!/usr/bin/python

import rospy
import baxter_interface
import pycuber
from pycuber.solver import CFOPSolver
from constant_parameters import right_prepare_position, joint_states, Joint_Names, new_joint_states
import kociemba

def joint2limb(i):
    right_move = {'right_s0':i[11], 'right_s1':i[12], 'right_e0':i[9], 'right_e1':i[10],
                  'right_w0':i[13], 'right_w1':i[14], 'right_w2':i[15]}
    left_move = {'left_s0':i[4], 'left_s1':i[5], 'left_e0':i[2], 'left_e1':i[3],
                 'left_w0':i[6], 'left_w1':i[7], 'left_w2':i[8]}
    return [right_move, left_move]

# Conversion to motion
def rotation2motion(rot):
    r2m_dict={D:
    }
    
    return

#Recognition
def recognition(camera_topic):
    kubic = "DRLUUBFBRBLURRLRUBLRDDFDLFUFUFFDBRDUBRUFLLFDDBFLUBLRBD"
    return kubic

#Solution
def solution(kubic):
    return kociemba.solve(kubic)
    
#Manipulation

def manipulation(sol):    
    limb_r = baxter_interface.Limb("right")
    limb_l = baxter_interface.Limb("left")
    sol = sol.split(" ")
    for i in sol:
        
    return 1

def main():
    rospy.init_node("rsm_kubic")
    kubic = recognition("")
    sol = solution(kubic)
    Manipulation(sol)
    
if __name__ == "__main__":
    main()
