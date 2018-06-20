import pybullet as p
import time
import math
import numpy as np
import random

p.connect(p.GUI)
p.loadURDF("pushing/plane.urdf",[0,0,-.2],globalScaling=6.0,useFixedBase=True)
# cylId = p.loadURDF("pushing/table/table.urdf",[0,0,0.2],globalScaling=6.0,useFixedBase=False)
cubeId = p.loadURDF("pushing/pr2_gripper.urdf",[2,2,0],useFixedBase=False)

t = 0
g = False

def eachIter():
	p.setGravity(0,0,-10)
	p.stepSimulation()
	time.sleep(.002)

while (1):
	eachIter()
	t+=1	
	