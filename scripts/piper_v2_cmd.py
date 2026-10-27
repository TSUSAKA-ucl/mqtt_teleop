#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK=1
import argcomplete
import argparse
import os
import sys

import time
# from dotenv import load_dotenv
from piper_sdk import *


def ctrl_disable(piper):
    while(piper.DisablePiper()):
        time.sleep(0.01)
    print("disabled")


def ctrl_enable(piper):
    time.sleep(0.1)
    while( not piper.EnablePiper()):
        time.sleep(0.01)
    print("enableing...")


def ctrl_go_zero(piper):
    factor = 57295.7795 #1000*180/3.1415926
    position = [0,0,0,0,0,0,0]
    
    joint_0 = round(position[0]*factor)
    joint_1 = round(position[1]*factor)
    joint_2 = round(position[2]*factor)
    joint_3 = round(position[3]*factor)
    joint_4 = round(position[4]*factor)
    joint_5 = round(position[5]*factor)
    joint_6 = round(position[6]*1000*1000)
    piper.ModeCtrl(0x01, 0x01, 30, 0x00)
    piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
    piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)


def ctrl_joint(piper):
    piper.GripperCtrl(0,1000,0x01, 0)
    factor = 57295.7795 #1000*180/3.1415926
    position = [0,0,0,0,0,0,0]
    count = 0
    while True:
        count  = count + 1
        # print(count)
        if(count == 0):
            print("1-----------")
            position = [0,0,0,0,0,0,0]
        elif(count == 300):
            print("2-----------")
            position = [0.2,0.2,-0.2,0.3,-0.2,0.5,0.08]
        elif(count == 600):
            print("1-----------")
            position = [0,0,0,0,0,0,0]
            count = 0
        
        joint_0 = round(position[0]*factor)
        joint_1 = round(position[1]*factor)
        joint_2 = round(position[2]*factor)
        joint_3 = round(position[3]*factor)
        joint_4 = round(position[4]*factor)
        joint_5 = round(position[5]*factor)
        joint_6 = round(position[6]*1000*1000)
        piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
        # print(piper.GetArmStatus())
        # print(position)
        time.sleep(0.005)


def ctrl_reset(piper):
    piper.MotionCtrl_1(0x02,0,0)#恢复


def ctrl_stop(piper):
    piper.MotionCtrl_1(0x01,0,0)


def read_joint_in_degree(piper):
    factor = 57295.7795 #1000*180/3.1415926
    inv_factor = 1/1000 #1/factor /Pi *180
    while True:
        msgs = piper.GetArmJointMsgs()
        jstate = msgs.joint_state
        # time stamp:1780639375.2284083
        # Hz:200.0
        # ArmMsgFeedBackJointStates:
        # Joint 1:-191
        # Joint 2:-692
        # Joint 3:-1314
        # Joint 4:0
        # Joint 5:22046
        # Joint 6:0
        joint1 = jstate.joint_1 * inv_factor
        joint2 = jstate.joint_2 * inv_factor
        joint3 = jstate.joint_3 * inv_factor
        joint4 = jstate.joint_4 * inv_factor
        joint5 = jstate.joint_5 * inv_factor
        joint6 = jstate.joint_6 * inv_factor
        joints = [joint1, joint2, joint3, joint4, joint5, joint6]
        print(joints)
        time.sleep(0.005)


def main():
    parser = argparse.ArgumentParser(description="Piper Control")
    parser.add_argument("command", choices=["enable", "disable", "go_zero", "joint", "reset", "stop", "read_joint"], help="Control command")
    parser.add_argument("--can_port", "-c", type=str, default=None, help="CAN port for PiPER (default: can0)")
    parser.add_argument(
        "-r", "--right_arm", action="store_true",
        help="Use right arm (default: False)"
    )
    parser.add_argument(
        "-l", "--left_arm", action="store_true",
        help="Use left arm (default: False)"
    )
    args = parser.parse_args()
    argcomplete.autocomplete(parser) 
    # if parser.parse_args().right_arm:
    #     can_port = "can_piper_r4e31"
    # elif parser.parse_args().left_arm:
    #     can_port = "can_piper_l282a"
    can_port = args.can_port
    if can_port is None:
        if parser.parse_args().right_arm:
            can_port = "can_piper_r4e31"
        elif parser.parse_args().left_arm:
            can_port = "can_piper_l282a"
        else:
            print("Please specify either --right_arm or --left_arm, or provide --can_port")
            sys.exit(1)
    piper = C_PiperInterface_V2(can_port)
    piper.ConnectPort()
    if args.command == "enable":
        ctrl_enable(piper)
    elif args.command == "disable":
        ctrl_disable(piper)
    elif args.command == "go_zero":
        ctrl_go_zero(piper)
    elif args.command == "joint":
        ctrl_joint(piper)
    elif args.command == "reset":
        ctrl_reset(piper)
    elif args.command == "stop":
        ctrl_stop(piper)
    elif args.command == "read_joint":
        read_joint_in_degree(piper)



if __name__ == "__main__":
    main()


