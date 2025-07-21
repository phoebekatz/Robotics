from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np
import sys

# This script makes the end-effector perform pick, pour, and place tasks
# Note that this script may not work for every arm as it was designed for the wx250
# Make sure to adjust commanded joint positions and poses as necessary
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250'
# Then change to this directory and type 'python bartender.py  # python3 bartender.py if using ROS Noetic'

def main():
    bot = InterbotixManipulatorXS("px100", "arm", "gripper")

    #Go to home
    bot.arm.set_single_joint_position("wrist_angle", 0)
    bot.arm.set_single_joint_position("elbow", 0)
    bot.arm.set_single_joint_position("shoulder", 0)
    
    #Go to the left and open gripper
    bot.arm.set_single_joint_position("waist", np.pi/2.0)
    bot.gripper.open()
    
    #Lower to floor and grip bottle
    bot.arm.set_single_joint_position("wrist_angle", -.85)
    bot.arm.set_single_joint_position("shoulder", 0.83)
    bot.gripper.close()
    
    #Lift
    bot.arm.set_single_joint_position("shoulder", 0.425)
    bot.arm.set_single_joint_position("wrist_angle", -.425)
    bot.arm.set_single_joint_position("shoulder", 0)
    bot.arm.set_single_joint_position("wrist_angle", 0)
    
    #Rotate to the right
    bot.arm.set_single_joint_position("waist", -np.pi/2.0)
    
    #Pour
    bot.arm.set_single_joint_position("shoulder", -0.23)
    bot.arm.set_single_joint_position("elbow", -0.35)
    bot.arm.set_single_joint_position("wrist_angle", 2.03)
    bot.arm.set_single_joint_position("shoulder", 0.4)
    
    #Unpour
    bot.arm.set_single_joint_position("shoulder", -0.23)
    bot.arm.set_single_joint_position("wrist_angle", 0)
    bot.arm.set_single_joint_position("elbow", 0)
    bot.arm.set_single_joint_position("shoulder", 0)
    
    #Rotate to the left
    bot.arm.set_single_joint_position("waist", np.pi/2.0)
    
    #Lower bottle to floor and release
    bot.arm.set_single_joint_position("wrist_angle", -.85)
    bot.arm.set_single_joint_position("shoulder", 0.83)
    bot.gripper.open()
    
    #Raise back to home position
    bot.arm.set_single_joint_position("shoulder", 0)
    bot.arm.set_single_joint_position("wrist_angle", 0)
    bot.arm.set_single_joint_position("waist", 0)

    #Go to sleep position
    bot.arm.set_single_joint_position("shoulder", -1.88)
    bot.arm.set_single_joint_position("elbow", 1.5)
    bot.arm.set_single_joint_position("wrist_angle", 0.8)

if __name__=='__main__':
    main()
