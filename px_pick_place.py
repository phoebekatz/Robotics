# copied from interbotix package, then edited

#PEK
# July 21 2025
# trying to directly switch px100 out for wx200

import time
import math

from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface

# This script uses a color/depth camera to get the arm to find objects and pick them up.
# For this demo, the arm is placed to the left of the camera facing outward. When the
# end-effector is located at x=0, y=-0.3, z=0.2 w.r.t. the 'wx200/base_link' frame, the AR
# tag should be clearly visible to the camera. A small basket should also be placed in front of the arm.
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_perception xsarm_perception.launch robot_model:=wx200'
# Then change to this directory and type 'python pick_place.py'

def main():
    # Initialize the arm module along with the pointcloud and armtag modules
    bot = InterbotixManipulatorXS("px100", moving_time=1.5, accel_time=0.75)
    pcl = InterbotixPointCloudInterface()
    armtag = InterbotixArmTagInterface()

    front_joint_positions = [0,0,0,0]
    waist_prelim = math.radians(-45)

    # set initial arm and gripper pose
    #bot.arm.set_ee_pose_components(x=0.5, z=0.5) #what are these numbers for px100? OG
    bot.arm.go_to_home_pose() 	
    bot.gripper.open()
    print("--pek start--")

    # get the ArmTag pose
    #bot.arm.set_ee_pose_components(y=-0.3, z=0.2) OG
    bot.arm.set_single_joint_position("waist", waist_prelim)
    time.sleep(0.5)
    armtag.find_ref_to_arm_base_transform()
    print("--pek error mark--")
    bot.arm.go_to_home_pose() 	
    #bot.arm.set_ee_pose_components(x=0.3, z=0.2)OG

    # get the cluster positions
    # sort them from max to min 'x' position w.r.t. the 'wx200/base_link' frame
    success, clusters = pcl.get_cluster_positions(ref_frame="px100/base_link", sort_axis="x", reverse=True)

    # pick up all the objects and drop them in a virtual basket in front of the robot
    for cluster in clusters:
        x, y, z = cluster["position"]
        print("x : {}".format(x))
        print("y : {}".format(y))
        print("z : {}".format(z))

        bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5) # right above cluster
        bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.5) # around cluster
        bot.gripper.close()
        time.sleep(2)
        bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5) # right above cluster
        #bot.arm.set_joint_positions(front_joint_positions,0.00001,0.0,0) # simultaneous motion
        #bot.arm.set_ee_pose_components(x=0.3, z=0.2) #OG code
        bot.arm.go_to_home_pose() 	
        bot.gripper.open()
        time.sleep(2)
    bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
