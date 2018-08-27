#!/usr/bin/python

import rospy
import baxter_interface
from baxter_interface import gripper
import pycuber
from pycuber.solver import CFOPSolver
from constant_parameters import right_prepare_position, joint_states, Joint_Names, new_joint_states
import kociemba
from math import pi

def joint2limb(i):
    right_move = {'right_s0':i[11], 'right_s1':i[12], 'right_e0':i[9], 'right_e1':i[10],
                  'right_w0':i[13], 'right_w1':i[14], 'right_w2':i[15]}
    left_move = {'left_s0':i[4], 'left_s1':i[5], 'left_e0':i[2], 'left_e1':i[3],
                 'left_w0':i[6], 'left_w1':i[7], 'left_w2':i[8]}
    return [right_move, left_move]

def grip_control(hand, cl_op):
    #Close = 1, open = 0
    grip = baxter_gripper.Gripper(hand)
    gripper.set_holding_force(50)
    gripper.set_dead_band(1)
    gripper.set_moving_force(80)
    if cl_op:
        gripper.close(block=True)
        print("Closed ", hand ," gripper!")
        rospy.sleep(1.0)
    else:
        gripper.open(block=True)
        print("Opened ", hand ," gripper!")
        rospy.sleep(1.0)

def parse_rot(rot):
    r2m_dict={"U":new_joint_states[2],
              "D":new_joint_states[4],
              "L":new_joint_states[6],
              "R":new_joint_states[0],
              "F":new_joint_states[7],
              "B":new_joint_states[1]
    }
    if "'" in rot:
        rot_ang = -pi/2
    elif "2" in rot:
        rot_ang = pi
    else:
        rot_ang = pi/2
    rot = rot[0]
    return [rot, rot_ang]

def move_limb(hand, move):
    limb = baxter_interface.Limb(hand)
    limb.move_to_joint_positions(move)
    print(hand, "limb moved")

def rotate_side(hand, rot_ang):
    limb = baxter_interface.Limb(hand)
    angles = limb.joint_angles()
    angles['right_w2'] = angles['right_w2'] + rot_ang
    limb.move_to_joint_positions(angles)
    print("Side rotated to ", rot_ang)
    
# Conversion to motion
def rotation2motion(rot):
    r = rospy.Rate(0.2)
    [rot, rot_ang] = parse_rot(rot)
    [right_move, left_move] = joint2limb(r2m_dict[rot])
    move_limb("left",left_move)
    move_limb("right", right_move)
    grip_control("right", 1)
    rotate_side("right", rot_ang)
    grip_control("right", 1)
    print "Waiting for next hand motion:"
    r.sleep
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
    sol = sol.split(" ")
    print("Put kubic in robot left hand.")
    input("Press Enter to continue...")
    grip_control("left", 1)
    grip_control("right", 0)
    for i in sol:
        rotation2motion(i)
    return 1

def main():
    rospy.init_node("rsm_kubic")
    kubic = recognition("")
    sol = solution(kubic)
    manipulation(sol)
    
if __name__ == "__main__":
    main()
