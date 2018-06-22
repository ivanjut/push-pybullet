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


"""
Defines an iteration through time
"""
def eachIter():
    p.setGravity(0,0,-10)
    p.stepSimulation()
    time.sleep(.005)


def applyAction(angle, dist, iters, orn):
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
    cube_height = p.getEulerFromQuaternion(cubeOrn)[2]
    print("Cube height: ", cube_height)
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

    line = p.addUserDebugLine(gripNewPos, [gripNewPos[0]-10*x_disp, gripNewPos[1]-10*y_disp, 0], lineColorRGB=(1, 0, 0)) # addUserDebugText
    print("Line id: ", line)

    # Execute push
    for i in range(iters):
        gripNewPos = [cubePos[0] + (iters-i)/(iters/dist) * x_disp, cubePos[1] + (iters-i)/(iters/dist) * y_disp,height]
        p.changeConstraint(cid, gripNewPos, p.getQuaternionFromEuler(new_orn))
        eachIter()
    for i in range(300):
        eachIter()

    p.removeUserDebugItem(line)

    # Back up gripper so no collisions
    for i in range(100):
        gripNewPos = [cubePos[0] + (i)/(100/dist) * x_disp, cubePos[1] + (i)/(100/dist) * y_disp,height]
        p.changeConstraint(cid, gripNewPos, p.getQuaternionFromEuler(new_orn))
        eachIter()
    for i in range(100):
        eachIter()


def distance(a, b):
    """
    Calculates the Euclidean distance between points a and b in 3D space
    :param a: first point as a 3D array 
    :param b: second point as a 3D array
    :return the Euclidean distance as a float
    """
    dist = math.sqrt(sum((b[0]-a[0])**2, (b[1]-a[1])**2, (b[2]-a[2])**2))
    return dist


def straight_line_loss():
    pass



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

        applyAction(angle, dist, iters, orn)

    eachIter()
    t+=1    
