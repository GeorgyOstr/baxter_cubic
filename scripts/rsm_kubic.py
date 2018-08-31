#!/usr/bin/python

#ROS+Baxter
import rospy
import baxter_interface
from baxter_interface import gripper

#Kociemba+pycuber
import pycuber
from pycuber.solver import CFOPSolver
import kociemba

#Constants
from math import pi
from joint_params import *

r2m_dict={"U":[lh_U_c, rh_U, 0],
          "D":[lh_D_c, rh_D, 0],
          "L":[lh_L, rh_L_c, 1],
          "R":[lh_R_c, rh_R, 0],
          "F":[lh_F, rh_F_c, 1],
          "B":[lh_B, rh_B_c, 1]
}

current_hand = 1 #Start with cubic in right hand

def grip_calibrate():
    print "Calibrating grippers..."
    grip_l = gripper.Gripper("left")
    grip_r = gripper.Gripper("right")
    grip_l.calibrate()
    grip_r.calibrate()
    print "Finished calibrating"

def grip_control(hand, cl_op):
    #Close = 1, open = 0
    while True:
        try:
            grip = gripper.Gripper(hand)
            break
        except:
            print("ERROR, can not connect to gripper!")
            rospy.sleep(1.0)
            print hand
            print("Attempt reconnecting...")
    grip.set_holding_force(50)
    grip.set_dead_band(1)
    grip.set_moving_force(80)
    if cl_op:
        grip.close(block=True)
        print("Closed " + hand +" gripper!")
        #rospy.sleep(1.0)
    else:
        grip.open(block=True)
        print("Opened " + hand +" gripper!")
        #rospy.sleep(1.0)

def parse_rot(rot):
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
    print(hand + " limb moved")

def rotate_side(hand, rot_ang):
    limb = baxter_interface.Limb(hand)
    angles = limb.joint_angles()
    angles[hand + '_w2'] = angles[hand + '_w2'] + rot_ang
    limb.move_to_joint_positions(angles)
    print("Side rotated to " + str(rot_ang))

def change_hand(right):
    global current_hand
    limb_r = baxter_interface.Limb("right")
    limb_l = baxter_interface.Limb("left")
    if right:
        print("Changing hand, r2l...")
        limb_r.move_to_joint_positions(rh_change_r2l_p)
        limb_l.move_to_joint_positions(lh_change_r2l_p)
        limb_r.move_to_joint_positions(rh_change_r2l)
        limb_l.move_to_joint_positions(lh_change_r2l)
        grip_control("left", 1)
        grip_control("right", 0)
        print("Hand changed! Now cubic in left hand.")
    else:
        print("Changing hand, l2r...")
        limb_r.move_to_joint_positions(rh_change_l2r_p)
        limb_l.move_to_joint_positions(lh_change_l2r_p)
        limb_r.move_to_joint_positions(rh_change_l2r)
        limb_l.move_to_joint_positions(lh_change_l2r)
        grip_control("left", 1)
        grip_control("right", 0)
        print("Hand changed! Now cubic in right hand.")
    current_hand = 1 - current_hand
    
# Conversion to motion
def rotation2motion(rot):
    global current_hand
    r = rospy.Rate(0.2)
    [rot, rot_ang] = parse_rot(rot)
    motion_mass = r2m_dict[rot]
    [left_move, right_move] = motion_mass[0:2]
    if motion_mass[2]!=current_hand:
        change_hand(current_hand)
    if current_hand:
        move_limb("left",left_move)
        move_limb("right", right_move)
        grip_control("left", 1)
        rotate_side("left", rot_ang)
        grip_control("left", 0)
    else:
        move_limb("left",left_move)
        move_limb("right", right_move)
        grip_control("right", 1)
        rotate_side("right", rot_ang)
        grip_control("right", 0)
    print "Returning to zero pos..."
    move_limb("left", lh_zero_pos)
    move_limb("right", rh_zero_pos)        
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
    print("Move to zero position.")
    raw_input("Press Enter to continue...")
    move_limb("left",lh_zero_pos)
    move_limb("right", rh_zero_pos)
    print("Prepare grippers for calibration.")
    raw_input("Press Enter to continue...")
    grip_calibrate()
    grip_control("left", 0)
    grip_control("right", 0)
    print("Put kubic in robot right hand.")
    raw_input("Press Enter to continue...")
    grip_control("left", 0)
    grip_control("right", 1)
    for c, i in enumerate(sol):
        print("Doing: " + str(i) + " Done: " + str(c) + "/" + str(len(sol)))
        rotation2motion(i)
    return 1

def main():
    rospy.init_node("rsm_kubic")
    kubic = recognition("")
    sol = solution(kubic)
    manipulation(sol)
    
if __name__ == "__main__":
    main()
