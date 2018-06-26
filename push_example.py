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
p.loadURDF("pushing/plane.urdf",[0,0,0], globalScaling=100.0, useFixedBase=True)
cube_id = p.loadURDF("pushing/cube.urdf", [2, 2, 1], useFixedBase=False)
grip_id = p.loadURDF("pushing/pr2_gripper.urdf", [0,0,4], globalScaling=4.0, useFixedBase=False)
constraint_id = p.createConstraint(grip_id, -1, -1, -1, p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,1])
num_joints = p.getNumJoints(grip_id)


def eachIter():
    """
    Defines an iteration through time
    """
    p.setGravity(0,0,-10)
    p.stepSimulation()
    time.sleep(.0001)


def apply_push(dist, iters, orn):
    """
    Applies a push to the cube with the corresponding parameters
    :param dist: the distance between gripper and cube (must be greater than radius of cube + radius of gripper)
    :param iters: the number of iterations the push will last
    :param orn: the orientation of the gripper in Euler angles, only x and y coordinates specified
    """
    assert dist >= MIN_DIST

    # Reset block every 100 pushes
    if (t % 100 == 0):
        p.resetBasePositionAndOrientation(grip_id, [0,0,4], p.getQuaternionFromEuler([0,0,0]))
        p.resetBasePositionAndOrientation(cube_id,[2,2,1],p.getQuaternionFromEuler([0, 0,random.uniform(0, 2*PI)]))

    cube_pos, cube_orn = p.getBasePositionAndOrientation(cube_id)
    x_disp = math.cos(p.getEulerFromQuaternion(cube_orn)[2])
    y_disp = math.sin(p.getEulerFromQuaternion(cube_orn)[2])

    # Pre push position for gripper
    new_grip_x, new_grip_y = cube_pos[0] + dist * x_disp, cube_pos[1] + dist * y_disp
    height = orn[1]*(2/PI) + 0.5
    grip_new_pos = [new_grip_x, new_grip_y, height]

    # Calculate new position/orientation of the gripper
    vec = np.array([cube_pos[0]-grip_new_pos[0], cube_pos[1]-grip_new_pos[1], 0])
    vec = vec / np.linalg.norm(vec)
    new_orn = [orn[0], orn[1], (math.atan2(vec[0], -vec[1]) - PI/2 )% (2*PI)]   # Points gripper at cube (z orientation)

    # Move gripper to new position
    for i in range(100):
        eachIter()
    p.changeConstraint(constraint_id, grip_new_pos, p.getQuaternionFromEuler(new_orn))
    for i in range(100):
        eachIter()
    p.setJointMotorControlArray(grip_id, range(num_joints), p.POSITION_CONTROL,[0.0]*num_joints)
    eachIter()

    # Draw line of desired trajectory
    start_x = grip_new_pos[0]
    start_y = grip_new_pos[1]
    end_x = grip_new_pos[0]-10*dist*x_disp
    end_y = grip_new_pos[1]-10*dist*y_disp
    line = p.addUserDebugLine(grip_new_pos, [end_x, end_y, 0], lineColorRGB=(1, 0, 0)) # addUserDebugText

    # Execute push and keep track of loss
    agg_straight_line_loss = 0
    for i in range(iters):
        new_grip_pos = [cube_pos[0] + (iters-i)/(iters/dist) * x_disp, cube_pos[1] + (iters-i)/(iters/dist) * y_disp, height]
        p.changeConstraint(constraint_id, new_grip_pos, p.getQuaternionFromEuler(new_orn))
        new_cube_pos, new_cube_orn = p.getBasePositionAndOrientation(cube_id)
        agg_straight_line_loss += straight_line_loss([start_x, start_y], [end_x, end_y], [new_cube_pos[0], new_cube_pos[1]])
        eachIter()
    for i in range(400):
        new_cube_pos, new_cube_orn = p.getBasePositionAndOrientation(cube_id)
        agg_straight_line_loss += straight_line_loss([start_x, start_y], [end_x, end_y], [new_cube_pos[0], new_cube_pos[1]])
        eachIter()
    new_cube_pos, new_cube_orn = p.getBasePositionAndOrientation(cube_id)
    ang_loss = angular_loss([start_x, start_y], [end_x, end_y], [new_cube_pos[0], new_cube_pos[1]])

    result = push_result([start_x, start_y], [end_x, end_y], [new_cube_pos[0], new_cube_pos[1]])

    # Print losses for the executed push
    # print("****************")
    # print("Aggregate Straight Line Loss: ", agg_straight_line_loss)
    # print("Mean Straight Line Loss: ", agg_straight_line_loss/iters)
    # print("Angular Loss: ", ang_loss)
    print("****************")
    print("Result: ", result)
    print("****************")
    p.removeUserDebugItem(line)

    # Data collection on Shakey
    # with open("/u0/home/ngothoskar/Desktop/out.txt", "a") as f:
    #     f.write(str(dist) + " " + str(iters) + " " + str(list(orn)) + "\n" + str(loss)+ "\n")

    # Data collection
    with open("test_output.txt", "a") as f:
        f.write(str(dist) + " " + str(iters) + " " + str(list(orn)) + "\n" + str(result)+ "\n")


    # Back up gripper so no collisions
    for i in range(100):
        new_grip_pos = [cube_pos[0] + (i)/(100/dist) * x_disp, cube_pos[1] + (i)/(100/dist) * y_disp,height]
        p.changeConstraint(constraint_id, new_grip_pos, p.getQuaternionFromEuler(new_orn))
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


def calc_angle(start, end, point):
    """
    Calculates the signed angle between the a line specified by (start, end) and a line specified by (start, point)
    :param start: the starting point of the trajectory
    :param end: the end point of the trajectory
    :param point: the object's point reference position
    :return: the angle between the two lines in radians [-PI, PI]
    """
    ref_vec = np.array([end[0] - start[0], end[1] - start[1]])
    obj_vec = np.array([point[0] - start[0], point[1] - start[1]])
    ref_mag = np.linalg.norm(ref_vec)
    obj_mag = np.linalg.norm(obj_vec)
    dot = np.dot(ref_vec, obj_vec)
    angle = math.acos(dot/(ref_mag * obj_mag))

    # Determine sign of angle
    mat = np.transpose(np.vstack((ref_vec, obj_vec)))
    det = np.linalg.det(mat)

    if det < 0:
        return angle * -1
    return angle


def push_result(start, end, point):
    """
    Calculates the result of the push as a tuple, component along straight line path, and offset from that path
    :param start: the starting point of the trajectory
    :param end: the end point of the trajectory
    :param point: the object's point reference position
    :return: a tuple containing the push result
    """
    angle = calc_angle(start, end, point)
    obj_vec = np.array([point[0] - start[0], point[1] - start[1]])
    obj_mag = np.linalg.norm(obj_vec)
    line_disp = obj_mag * math.degrees(math.cos(abs(angle)))
    offset = obj_mag * math.degrees(math.sin(angle))
    return line_disp, offset


# Run random samples
while (1):
    for i in range(1000):
        dist = random.uniform(2, 5)
        iters = random.randint(200, 600)
        orn = [random.uniform(0, 2 * PI), random.uniform(0, PI/2)]
        # print("Distance: ", dist)
        # print("Iterations: ", iters)
        # print("Orientation: ", orn)
        apply_push(dist, iters, orn)
        t+=1

    eachIter()
    
