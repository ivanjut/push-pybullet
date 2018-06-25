from __future__ import division, print_function
import pybullet as p
import time
import math
import numpy as np
import random


PI = math.pi
MIN_DIST = 2
t = 0

p.connect(p.GUI)
p.loadURDF("pushing/plane.urdf",[0,0,0], globalScaling=1.0, useFixedBase=True)
cubeId = p.loadURDF("pushing/cube.urdf", [2, 2, 1], useFixedBase=False)
gripId = p.loadURDF("pushing/pr2_gripper.urdf", [0,0,4], globalScaling=4.0, useFixedBase=False)
cid = p.createConstraint(gripId, -1, -1, -1, p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,1])
numJoints = p.getNumJoints(gripId)


def eachIter():
    """
    Defines an iteration through time
    """
    p.setGravity(0,0,-10)
    p.stepSimulation()
    time.sleep(.001)


def apply_push(angle, dist, iters, orn):
    """
    Applies a push to the cube with the corresponding parameters
    :param angle: the angle of push relative to the world frame of the gripper
    :param dist: the distance between gripper and cube (must be greater than radius of cube + radius of gripper)
    :param iters: the number of iterations the push will last
    :param orn: the orientation of the gripper in Euler angles, only x and y coordinates specified
    """
    assert dist >= MIN_DIST

    gripPos, gripOrn = p.getBasePositionAndOrientation(gripId)
    cubePos, cubeOrn = p.getBasePositionAndOrientation(cubeId)
    x_disp = math.cos(math.radians(angle))
    y_disp = math.sin(math.radians(angle))

    # Pre push position for gripper with collision avoidance
    newGripX, newGripY = cubePos[0] + dist * x_disp, cubePos[1] + dist * y_disp
    preGripNewPos = [newGripX, newGripY, 4]
    height = orn[1]*(2/PI) + 0.5
    gripNewPos = [newGripX, newGripY, height]

    # Calculate new position/orientation of the gripper
    vec = np.array([cubePos[0]-gripNewPos[0], cubePos[1]-gripNewPos[1], 0])
    vec = vec / np.linalg.norm(vec)
    new_orn = [orn[0], orn[1], (math.atan2(vec[0], -vec[1]) - PI/2 )% (2*PI)]   # Points gripper at cube (z orientation)

    # Set height of gripper to be above cube so there is no collision when it readjusts
    p.changeConstraint(cid, [gripPos[0],gripPos[1], 4], p.getQuaternionFromEuler(new_orn))
    for i in range(100):
        eachIter()
    eachIter()

    # Move gripper to new position
    p.changeConstraint(cid, preGripNewPos, p.getQuaternionFromEuler(new_orn))
    for i in range(100):
        eachIter()
    p.changeConstraint(cid, gripNewPos, p.getQuaternionFromEuler(new_orn))
    for i in range(100):
        eachIter()
    p.setJointMotorControlArray(gripId, range(numJoints), p.POSITION_CONTROL,[0.0]*numJoints)
    eachIter()

    # Draw line of desired trajectory
    start_x = gripNewPos[0]
    start_y = gripNewPos[1]
    end_x = gripNewPos[0]-10*dist*x_disp
    end_y = gripNewPos[1]-10*dist*y_disp
    line = p.addUserDebugLine(gripNewPos, [end_x, end_y, 0], lineColorRGB=(1, 0, 0)) # addUserDebugText

    # Execute push and keep track of loss
    agg_straight_line_loss = 0
    for i in range(iters):
        new_grip_pos = [cubePos[0] + (iters-i)/(iters/dist) * x_disp, cubePos[1] + (iters-i)/(iters/dist) * y_disp,height]
        p.changeConstraint(cid, new_grip_pos, p.getQuaternionFromEuler(new_orn))
        new_cube_pos, new_cube_orn = p.getBasePositionAndOrientation(cubeId)
        agg_straight_line_loss += straight_line_loss([start_x, start_y], [end_x, end_y], [new_cube_pos[0], new_cube_pos[1]])
        eachIter()
    for i in range(400):
        new_cube_pos, new_cube_orn = p.getBasePositionAndOrientation(cubeId)
        agg_straight_line_loss += straight_line_loss([start_x, start_y], [end_x, end_y], [new_cube_pos[0], new_cube_pos[1]])
        eachIter()
    new_cube_pos, new_cube_orn = p.getBasePositionAndOrientation(cubeId)
    ang_loss = angular_loss([start_x, start_y], [end_x, end_y], [new_cube_pos[0], new_cube_pos[1]])

    # Print losses for the executed push
    print("****************")
    print("Aggregate Straight Line Loss: ", agg_straight_line_loss)
    print("Mean Straight Line Loss: ", agg_straight_line_loss/iters)
    print("Angular Loss: ", ang_loss)
    print("****************")
    p.removeUserDebugItem(line)

    # Back up gripper so no collisions
    for i in range(100):
        new_grip_pos = [cubePos[0] + (i)/(100/dist) * x_disp, cubePos[1] + (i)/(100/dist) * y_disp,height]
        p.changeConstraint(cid, new_grip_pos, p.getQuaternionFromEuler(new_orn))
        eachIter()
    for i in range(100):
        eachIter()


def distance_point_line(start, end, point):
    """
    Calculates the perpendicular distance from a point to a line in 2D
    :param start: the starting point of the line
    :param end: the end point of the line
    :param point: the point in question
    :return the distance between point and the line segment formed by start/end
    """
    x_diff, y_diff = end[0] - start[0], end[1] - start[1]
    point_diff_x, point_diff_y = start[0] - point[0], start[1] - point[1]
    norm = math.sqrt(x_diff**2 + y_diff**2)
    dist = abs((x_diff * point_diff_y) - (point_diff_x * y_diff))/norm
    return dist


def straight_line_loss(start, end, point):
    """
    Squared distance loss function to straight line trajectory
    :param start: the starting point of the trajectory
    :param end: the end point of the trajectory
    :param point: the object's point reference position
    :return: the squared distance between the point and the straight line trajectory
    """
    return distance_point_line(start, end, point)**2


def angular_loss(start, end, point):
    """
    Angular loss function between the object and the straight line trajectory based on the starting reference point
    :param start: the starting point of the trajectory
    :param end: the end point of the trajectory
    :param point: the object's point reference position
    :return: the squared angle in radians between the object and the trajectory with reference to the starting point
    """
    ref_vec = (end[0] - start[0], end[1] - start[1])
    obj_vec = (point[0] - start[0], point[1] - start[1])
    ref_mag = math.sqrt(ref_vec[0]**2 + ref_vec[1]**2)
    obj_mag = math.sqrt(obj_vec[0]**2 + obj_vec[1]**2)
    dot = ref_vec[0] * obj_vec[0] + ref_vec[1] * obj_vec[1]
    cos = dot/(ref_mag * obj_mag)
    angle = math.degrees(math.acos(cos))
    return angle ** 2


# Run random samples
while (1):
    for i in range(1000):
        angle = random.randint(0, 360)
        dist = random.uniform(2, 5)
        iters = random.randrange(200, 600, 100)
        orn = [random.uniform(0, 2 * PI), random.uniform(0, PI/2)]
        print("Angle: ", angle)
        print("Distance: ", dist)
        print("Iterations: ", iters)
        print("Orientation: ", orn)

        apply_push(angle, dist, iters, orn)

    eachIter()
    t+=1
