#!/usr/bin/python

#Parser
import argparse
import os, sys, json
parser = argparse.ArgumentParser("Rubiks Square Extractor")
parser.add_argument('-nc', '--nocalibration', type=bool, help='Run without calibration', default=False)
parser.add_argument('-w', '--webcam', type=int, default=None, help='webcam to use...0, 1, etc')
parser.add_argument('-d', '--delay', type=int, default=None, help='Delay for recognition in sec: 5, 10, 15 etc')

args = parser.parse_args()

#Dwalton cubic tracker and resolver
from rubikscubetracker import RubiksVideo, RubiksImage, merge_two_dicts
from threading import Thread
import subprocess #For resolver
import shlex
from multiprocessing import Pool    

#ROS+Baxter
import rospy
import baxter_interface
from baxter_interface import gripper

#Kociemba+pycuber
import pycuber
from pycuber.solver import CFOPSolver
import kociemba

#Constants
from math import pi, sqrt
from joint_params import *

#Recognition
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


r2m_dict={"U":[lh_U_c, rh_U, 0],
          "D":[lh_D_c, rh_D, 0],
          "L":[lh_L, rh_L_c, 1],
          "R":[lh_R_c, rh_R, 0],
          "F":[lh_F, rh_F_c, 1],
          "B":[lh_B, rh_B_c, 1]
}

current_hand = 1 #Start with cubic in right hand
take_picture = ""



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
        except Exception,e:
            print("ERROR, can not connect to gripper!")
            print e
            rospy.sleep(1.0)
            print hand
            print("Attempt reconnecting...")
    grip.set_holding_force(50)
    grip.set_dead_band(1)
    grip.set_moving_force(80)
    if cl_op:
        grip.close(block=True)
        print("Closed " + hand +" gripper!")
        rospy.sleep(1.0)
    else:
        grip.open(block=True)
        print("Opened " + hand +" gripper!")
        rospy.sleep(1.0)

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
        move_limb("left", lh_zero_pos)
        move_limb("right", rh_zero_pos)       
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
    #r.sleep
    return

#Recognition
def get_pictures_of_sides():
    global take_picture
    sl_time = args.delay;
    print "Saving rubiks cube side pictures..."
    move_limb("right",r_camera_move_L)
    rospy.sleep(sl_time)        
    move_limb("right",r_camera_move_F)
    rospy.sleep(sl_time)
    move_limb("right",r_camera_move_B)
    rospy.sleep(sl_time)
    print("Move to zero position.")
    move_limb("left",lh_zero_pos)
    move_limb("right", rh_zero_pos)                  
    change_hand(1)
    print("Move to zero position.")
    move_limb("left",lh_zero_pos)
    move_limb("right", rh_zero_pos)
    move_limb("left",l_camera_move_U)
    rospy.sleep(sl_time)
    move_limb("left",l_camera_move_R)
    rospy.sleep(sl_time)
    move_limb("left",l_camera_move_D)
    rospy.sleep(sl_time)
    print "All sides pictures taken! Analyzing..."

def run_command(command):
    process = subprocess.Popen(shlex.split(command), stdout=subprocess.PIPE)
    while True:
        output = process.stdout.readline()
        if output == '' and process.poll() is not None:
            break
        if output:
            data = output.strip()
    rc = process.poll()
    print "Kociemba is:", data
    return data

def run_recog():
    global kubic_return
    kubic_return = "Not enough time!!!"
    
    if args.webcam is not None:
        rvid = RubiksVideo(args.webcam)
        kubic = rvid.analyze_webcam()
        
    print "thread finished...exiting"                
    print "Cubic recognized!"
    print kubic
    kubic_return = kubic
    return kubic

def recognition():
    global kubic_return
    Thread(target = get_pictures_of_sides).start()
    Thread(target = run_recog).start()
    kubic = kubic_return
    kubic_fix = raw_input("Please insert the right kubic configuration if it did not recognized it correctly!")
    if len(kubic_fix)<len("DRLUUBFBRBLURRLRUBLRDDFDLFUFUFFDBRDUBRUFLLFDDBFLUBLRBD"):
        print("There was no fix...")   
        return kubic
    else:
        print("The cube is fixed a bit: ")
        print(kubic_fix)
        return kubic_fix

#Solution
def solution(kubic):
    print "Searching solution..."
    sol = kociemba.solve(kubic)
    print sol
    return sol
    
#Manipulation

def manipulation(sol):
    sol = sol.split(" ")
    for c, i in enumerate(sol):
        print("Doing: " + str(i) + " Done: " + str(c) + "/" + str(len(sol)))
        rotation2motion(i)
    return 1

def calib_init():
    print("Move to zero position.")
    raw_input("Press Enter to continue...")
    move_limb("left",lh_zero_pos)
    move_limb("right", rh_zero_pos)
    if not args.nocalibration:
        print("Prepare grippers for calibration.")
        raw_input("Press Enter to continue...")
        grip_calibrate()
    grip_control("left", 0)
    grip_control("right", 0)
    print("Put kubic in robot right hand.")
    raw_input("Press Enter to continue...")
    grip_control("left", 0)
    grip_control("right", 1)

def main():
    rospy.init_node("rsm_kubic")
    print("This node performs recognition, solution, and manipulation of cubic rubic...")
    calib_init()
    kubic = recognition()
    sol = solution(kubic)
    print "Found solution! Manipulating!!!"
    manipulation(sol)
    
if __name__ == "__main__":
    main()
