import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print ("current_dir=" + currentdir)
parentdir = os.path.join(currentdir,"../gym")

os.sys.path.insert(0,parentdir)

import pybullet as p
import pybullet_data
import time



#########---------------START Obstacle Avoidance----------########


import math
import numpy as np
import random


def find_largest_gap(collisions):
	c = np.asarray(collisions) < 0 # true where gap exists
	d = np.insert(np.append(c, [False]), 0, False) #inserts false beggining/end
	f = np.nonzero(np.diff(d))[0] #finds start/finish indices of gaps
	g = f[1::2] - f[0::2] #gap sizes

	if g.size == 0:
		return None

	i = np.argmax(g) #largest gap
	l = g[i] #length
	s = f[2*i] #start index

	return int(s + l/2)



#########---------------END Obstacle Avoidance----------########



#########---------------START Random Obstacles----------########



def random_environment(blocks, r):
	for i in range(blocks):
		rand = list(range(-r,0)) + list(range(1, r))
		randlist = [random.choice(rand), random.choice(rand),0]
		p.createMultiBody (0,cube, baseOrientation=orn, basePosition=randlist)



#########---------------END Random Obstacles----------########



cid = p.connect(p.SHARED_MEMORY)
if (cid<0):
	p.connect(p.GUI)
	
p.resetSimulation()
p.setGravity(0,0,-10)

useRealTimeSim = 1



cube =p.createCollisionShape(p.GEOM_MESH,fileName="cube.obj",flags=p.GEOM_FORCE_CONCAVE_TRIMESH, meshScale=[1,1,1])
orn = p.getQuaternionFromEuler([0,0,0])
print(orn)



random_environment(20, 3)



#for video recording (works best on Mac and Linux, not well on Windows)
#p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "racecar.mp4")
p.setRealTimeSimulation(useRealTimeSim) # either this
#p.loadURDF("plane.urdf")
p.loadSDF(os.path.join(pybullet_data.getDataPath(),"stadium.sdf"))

car = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"racecar/racecar.urdf"))
for i in range (p.getNumJoints(car)):
	print (p.getJointInfo(car,i))

inactive_wheels = [3,5,7]
wheels = [2]

for wheel in inactive_wheels:
		p.setJointMotorControl2(car,wheel,p.VELOCITY_CONTROL,targetVelocity=0,force=0)
	
steering = [4,6]

targetVelocitySlider = p.addUserDebugParameter("wheelVelocity",-10,10,0)
maxForceSlider = p.addUserDebugParameter("maxForce",0,10,10)
steeringSlider = p.addUserDebugParameter("steering",-0.5,0.5,0)
while (True):
	maxForce = p.readUserDebugParameter(maxForceSlider)
	targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
	steeringAngle = p.readUserDebugParameter(steeringSlider)
	


#########---------------START Obstacle Avoidance----------########


	ray_length = 1
	angle_swept = 130
	step = math.ceil(100*angle_swept/p.MAX_RAY_INTERSECTION_BATCH_SIZE)/100
	angles = np.arange(-angle_swept/2, angle_swept/2, step) * np.pi / 180 #angle of rotations
	num_rays = np.shape(angles)[0]
	rays = np.concatenate(([ray_length*np.sin(angles)], [ray_length*np.cos(angles)], [np.zeros(num_rays)]), axis=0)
	rot = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]])
	offset = np.array([0, 0, 0.3])

	position, orientation = p.getBasePositionAndOrientation(car)

	matrix = p.getMatrixFromQuaternion(orientation)
	matrix = np.reshape(matrix, (3, 3))

	src = np.array(position) 
	src = src + np.matmul(matrix,offset)

	rays_src = np.repeat([src], num_rays, axis=0)

	orn = np.matmul(matrix, rot) #rotates unit vector y to -x
	rays_end = np.matmul(orn, rays) # unit vector in direction of minitaur

	rays_end = (rays_end + src[:, None]).T
	rays_info = p.rayTestBatch(rays_src.tolist(), rays_end.tolist())

	b = [int(i[0]) for i in rays_info]
	nth_ray = find_largest_gap(b)


	if(nth_ray == None):
		targetVelocity = 0
	else:
		deg = angles[nth_ray]*180/np.pi
		print("Rotate {:.1f} degrees".format(deg))
		if math.fabs(deg)  > 5:
			steeringAngle = np.sign(deg)*1


#########---------------END Obstacle Avoidance----------########


	

	
	for wheel in wheels:
		p.setJointMotorControl2(car,wheel,p.VELOCITY_CONTROL,targetVelocity=targetVelocity,force=maxForce)
		
	for steer in steering:
		p.setJointMotorControl2(car,steer,p.POSITION_CONTROL,targetPosition=steeringAngle)
		
	steering
	if (useRealTimeSim==0):
		p.stepSimulation()
	time.sleep(0.01)