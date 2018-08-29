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

def grip_calibrate():
    print "Calibrating grippers..."
    grip_l = gripper.Gripper("left")
    grip_r = gripper.Gripper("right")
    grip_l.calibrate()
    grip_r.calibrate()
    print "Finished calibrating"

def grip_control(hand, cl_op):
    #Close = 1, open = 0
    grip = gripper.Gripper(hand)
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
    
# Conversion to motion
def rotation2motion(rot):
    r = rospy.Rate(0.2)
    [rot, rot_ang] = parse_rot(rot)
    [left_move, right_move] = r2m_dict[rot][0:2]

    move_limb("left",left_move)
    move_limb("right", right_move)
    grip_control("left", 1)
    rotate_side("left", rot_ang)
    grip_control("left", 0)
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
