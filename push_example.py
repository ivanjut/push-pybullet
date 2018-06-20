#!/usr/bin/env python

from __future__ import print_function

import pybullet as p
import time
import math
import numpy as np
import random


from ss_pybullet.pybullet_tools.pr2_utils import TOP_HOLDING_LEFT_ARM, \
    SIDE_HOLDING_LEFT_ARM, PR2_GROUPS, open_arm, get_disabled_collisions, REST_LEFT_ARM, rightarm_from_leftarm
from ss_pybullet.pybullet_tools.utils import set_base_values, joint_from_name, set_joint_position, \
    set_joint_positions, add_data_path, connect, plan_base_motion, plan_joint_motion, enable_gravity, input, \
    joint_controller, dump_body, load_model, joints_from_names, get_joint_name, get_joints, user_input, disconnect


"""
Moving base motion planning
"""
def test_base_motion(pr2, base_start, base_goal):
    #disabled_collisions = get_disabled_collisions(pr2)
    set_base_values(pr2, base_start)
    user_input('Plan Base?')
    base_limits = ((-2.5, -2.5), (2.5, 2.5))
    base_path = plan_base_motion(pr2, base_goal, base_limits)
    if base_path is None:
        print('Unable to find a base path')
        return
    print(len(base_path))
    for bq in base_path:
        set_base_values(pr2, bq)
        # user_input('Continue?')
        time.sleep(0.05)


def test_arm_motion(pr2, left_joints, arm_goal):
    disabled_collisions = get_disabled_collisions(pr2)
    user_input('Plan Arm?')
    arm_path = plan_joint_motion(pr2, left_joints, arm_goal, disabled_collisions=disabled_collisions)
    if arm_path is None:
        print('Unable to find an arm path')
        return
    print(len(arm_path))
    for q in arm_path:
        set_joint_positions(pr2, left_joints, q)
        #raw_input('Continue?')
        time.sleep(0.01)


def test_arm_control(pr2, left_joints, arm_start):
    user_input('Control Arm?')
    real_time = False
    enable_gravity()
    p.setRealTimeSimulation(real_time)
    for _ in joint_controller(pr2, left_joints, arm_start):
        if not real_time:
            p.stepSimulation()
        #time.sleep(0.01)

"""
Initialize the PR2 arms to a useful configuration
"""
def put_arms_in_useful_configuration(pr2):
    arm_start = SIDE_HOLDING_LEFT_ARM
    arm_goal = TOP_HOLDING_LEFT_ARM

    left_joints = [joint_from_name(pr2, name) for name in PR2_GROUPS['left_arm']]
    right_joints = [joint_from_name(pr2, name) for name in PR2_GROUPS['right_arm']]
    set_joint_positions(pr2, left_joints, arm_start)
    set_joint_positions(pr2, right_joints, rightarm_from_leftarm(REST_LEFT_ARM))
    set_joint_position(pr2, joint_from_name(pr2, 'torso_lift_joint'), 0.2)
    open_arm(pr2, 'left')



"""
Does one iteration of push
"""
def apply_push(obj, pr2, angle, force):
    objPos, objOrn = p.getBasePositionAndOrientation(obj)
    print("Object position: ", objPos)
    print("Object orientation: ", objOrn)

    print("Joint: ", joint_from_name(pr2, "r_gripper_l_finger_joint"))


    # To disable motor in order to use direct torque control
    # p.setJointMotorControl2(obj, jointIndex=0,controlMode=p.VELOCITY_CONTROL, force=0)

    # Direct torque control
    p.setJointMotorControl2(bodyUniqueId=pr2, jointIndex=0, controlMode=p.TORQUE_CONTROL,force=force)


#####################################

def main():
    # connect(use_gui=True)
    p.connect(p.GUI)
    add_data_path()

    # Load objects
    plane = p.loadURDF("plane.urdf")
    table_path = "/ss_pybullet/models/table_collision/table.urdf"
    table = p.loadURDF(table_path, 0, 0, 0, 0, 0, 0.707107, 0.707107)
    cube = p.loadURDF("pushing/cube.urdf", [0,0,0.65], globalScaling = 0.1)

    # Initialize PR2 and world
    pr2_urdf = "/ss_pybullet/models/pr2_description/pr2.urdf"
    pr2_start_orientation = p.getQuaternionFromEuler([0,0,0])
    pr2_start_pose = [-0.65,0,0]
    pr2 = p.loadURDF(pr2_urdf, pr2_start_pose, pr2_start_orientation, useFixedBase=True )
    put_arms_in_useful_configuration(pr2)
    enable_gravity()

    # apply_push(cube, pr2, 5, 500)


    # base_start = (-1, 0, 0)
    # base_goal = (2, 2, 0)

    # p.addUserDebugLine(base_start, base_goal, lineColorRGB=(1, 0, 0)) # addUserDebugText
    # print(base_start, base_goal)
    
    # test_base_motion(pr2, base_start, base_goal)
    # test_arm_motion(pr2, left_joints, arm_goal)
    # test_arm_control(pr2, left_joints, arm_start)

    user_input('Finish?')
    disconnect()

if __name__ == '__main__':
    main()