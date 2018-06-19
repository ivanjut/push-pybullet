import pybullet as p
import time
import math
import numpy as np
import random

p.connect(p.GUI)
p.loadURDF("plane.urdf",[0,0,-.2],globalScaling=1000.0,useFixedBase=True)
cylId = p.loadURDF("simple_cylinder.urdf",[0,0,0.2],globalScaling=6.0,useFixedBase=False)
cubeId = p.loadURDF("cube.urdf",[2,2,0],globalScaling=0.6,useFixedBase=False)

t = 0
g = False

def eachIter():
	p.setGravity(0,0,-10)
	p.stepSimulation()
	time.sleep(.0001)
	
def increment(tup, ix, val):
	temp = list(tup)
	temp[ix] += val
	return tuple(temp)

def applyAction(angle,offset):
	cylPos, cylOrn = p.getBasePositionAndOrientation(cylId)
	cubePos, cubeOrn = p.getBasePositionAndOrientation(cubeId)
	cubeNewPos = [cylPos[0] + math.cos(math.radians(angle)),cylPos[1] + math.sin(math.radians(angle)),0]
	vec = np.array([cylPos[0]-cubeNewPos[0],cylPos[1]-cubeNewPos[1],0])
	vec = vec / np.linalg.norm(vec)
	look = [0,0,math.atan(vec[0]/(-vec[1]))]
	cubeNewPosWithOffset = cubeNewPos
	cubeNewPosWithOffset[0] -= offset*math.sin(math.radians(angle))
	cubeNewPosWithOffset[1] += offset*math.cos(math.radians(angle))
	p.resetBasePositionAndOrientation(cubeId,cubeNewPosWithOffset,p.getQuaternionFromEuler(look))

	for i in range(100):
		eachIter()

	for i in range(100):
		p.applyExternalForce(cubeId, -1, 20*np.array(vec), cubeNewPos, flags = p.WORLD_FRAME)
		eachIter()

	for i in range(400):
		eachIter()
	
	cubePos, cubeOrn = p.getBasePositionAndOrientation(cubeId)
	print angle, offset
	with open("/u0/home/ngothoskar/Desktop/out.txt", "a") as text_file:
		text_file.write("%d %f\n%s\n" % (angle,offset, str(list(np.array(list(cubePos)) - cubeNewPosWithOffset))))

while (1):

	# a = [int(x) for x in raw_input().split()]
	angle = random.randint(0,360)
  	offset = random.uniform(-0.3, 0.3)

	for i in range(1000):
		applyAction(angle, offset)
	# applyAction(*a)
	eachIter()
	t+=1	
	