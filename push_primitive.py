import pybullet as p
import time
import math
import numpy as np
import random

p.connect(p.GUI)
p.loadURDF("pushing/plane.urdf",[0,0,0], globalScaling=1.0, useFixedBase=True)
cubeId = p.loadURDF("pushing/cube.urdf", [2,2,0.3], useFixedBase=False)
gripId = p.loadURDF("pushing/pr2_gripper.urdf", [0,0,6], globalScaling=4.0, useFixedBase=False)
cid = p.createConstraint(gripId, -1, -1, -1, p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,1])
numJoints = p.getNumJoints(gripId)


t = 0
g = False

def eachIter():
	p.setGravity(0,0,-10)
	p.stepSimulation()
	time.sleep(.005)

def applyAction(angle):
	gripPos, gripOrn = p.getBasePositionAndOrientation(gripId)
	cubePos, cubeOrn = p.getBasePositionAndOrientation(cubeId)
	p.changeConstraint(cid, [gripPos[0],gripPos[1],6], p.getQuaternionFromEuler([0,0,0]))
	eachIter()
	gripNewPos = [cubePos[0] + 4*math.cos(math.radians(angle)),cubePos[1] + 4*math.sin(math.radians(angle)),0.5]

	vec = np.array([cubePos[0]-gripNewPos[0],cubePos[1]-gripNewPos[1],0])
	vec = vec / np.linalg.norm(vec)

	look = [0,0,(math.atan2(vec[0],-vec[1]) - math.pi/2 )% (2*math.pi)]

	p.changeConstraint(cid, gripNewPos, p.getQuaternionFromEuler(look))
	p.setJointMotorControlArray(gripId, range(numJoints), p.POSITION_CONTROL,[0.0]*numJoints)
	eachIter()


	for i in range(600):
		gripNewPos = [cubePos[0] + (600-i)/120.0*math.cos(math.radians(angle)),cubePos[1] + (600-i)/120.0*math.sin(math.radians(angle)),0.5]
		p.changeConstraint(cid, gripNewPos, p.getQuaternionFromEuler(look))
		eachIter()

	for i in range(300):
		eachIter()


while (1):
	for i in range(1000):
		angle = random.randint(0,360)
		applyAction(angle)

	eachIter()
	t+=1	
