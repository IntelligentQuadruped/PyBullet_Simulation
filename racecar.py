import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print ("current_dir=" + currentdir)
parentdir = os.path.join(currentdir,"../gym")

os.sys.path.insert(0,parentdir)

import pybullet as p
import pybullet_data
import time
import csv

#########---------------START Obstacle Avoidance----------########

import math
import numpy as np
import random
import matplotlib.pyplot as plt

class __Path:
	def __init__(self):
		self.len = 0
		self.curr = None
		self.rad = 0


	def calcDist(self, pos):
		if self.curr is not None:
			prev = self.curr
			self.curr = pos
			dist = np.linalg.norm(pos-prev)
			self.len = self.len + dist
			self.rad = np.linalg.norm(pos-self.start)
		else:
			self.start = pos
			self.curr = pos





class __Data:

	def __init__(self, row_i, start_i, end_i, n):
		self.row_i = row_i
		self.start_i = start_i
		self.end_i = end_i
		self.n = n
		self.gap = None

	def compareGap(self, s, f):

		g = self.gap
		if g is not None:
			return g[1]-g[0] < f-s
		else:
			return True

	def setGap(self, s, f):

		g = self.gap
		if g is None:
			self.gap = (s, f)
		else:
			if f-s > g[1]-g[0]:
				self.gap = (s, f)


def __addNextRow(row, start, finish, data):

	if row == data.n:
		data.setGap(start, finish)
		return
	args = np.argwhere(data.row_i == row)
	for i in args:
		s = start
		f = finish
		c = data.start_i[i][0]
		d = data.end_i[i][0]
		if s < d and f > c:
			if s < c:
				s = c
			if f > d:
				f = d
			if data.compareGap(s, f):
				__addNextRow(row+1, s, f, data)
		return

 
def find_largest_gap(collisions):
	depth =  collisions < 0 # true where gap exists
	npad = ((0, 0), (1, 1))
	d = np.pad(depth, pad_width=npad, mode='constant', constant_values=0)

	f = np.nonzero(np.diff(d))
	r = f[0][0::2] # row indices
	data = __Data(r, f[1][0::2], f[1][1::2], len(np.unique(r)))

	__addNextRow(0, 0, np.inf, data)

	sf = data.gap
	if sf is None or sf[1] == np.inf:
		return None

	return (sf[0]+sf[1])/2


#########---------------END Obstacle Avoidance----------########

#########---------------START Random Obstacles----------########

def random_environment(blocks, r):
	ran = list(range(-r,1)) + list(range(1, r+1))
	R1, R2 = np.meshgrid(ran, ran)
	idx = np.random.permutation(R1.size)
	r1 = R1.flatten()[idx]
	r2 = R2.flatten()[idx]
	
	for i in range(blocks):
		randlist = [r1[i], r2[i],0]
		if all(abs(i) < 2 for i in randlist):
			continue
		p.createMultiBody(0,cube, baseOrientation=orn, basePosition=randlist)

#########---------------END Random Obstacles----------########

with open('rc_output.csv', 'w+') as f:
	writer = csv.writer(f, quoting=csv.QUOTE_NONNUMERIC)
	writer.writerow( ('Distance Travelled', 'Radial DIstance', 'Deadend', 'Blocks', 'Radius') )

	while True:

		cid = p.connect(p.SHARED_MEMORY)
		if (cid<0):
			# p.connect(p.GUI)
			p.connect(p.DIRECT)

		p.setPhysicsEngineParameter(numSolverIterations=5, fixedTimeStep=1., 
									numSubSteps=50)
			
		p.resetSimulation()
		p.setGravity(0,0,-10)

		useRealTimeSim = False

		cube =p.createCollisionShape(p.GEOM_MESH,fileName=os.path.join(pybullet_data.getDataPath(),"cube.onj"),flags=p.GEOM_FORCE_CONCAVE_TRIMESH, meshScale=[1,1,1])
		orn = p.getQuaternionFromEuler([0,0,0])

		radius = 10
		blocks = 125
		random_environment(blocks, radius)


		p.setRealTimeSimulation(useRealTimeSim) # either this
		p.loadURDF(os.path.join(pybullet_data.getDataPath(),"plane.urdf"))

		car = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"racecar/racecar.urdf"))
		for i in range (p.getNumJoints(car)):
			print (p.getJointInfo(car,i))

		inactive_wheels = [5,7]
		wheels = [2,3]

		for wheel in inactive_wheels:
				p.setJointMotorControl2(car,wheel,p.VELOCITY_CONTROL,targetVelocity=0,force=0)
			
		steering = [4,6]

		targetVelocitySlider = p.addUserDebugParameter("wheelVelocity",-10,10,0)
		maxForceSlider = p.addUserDebugParameter("maxForce",0,10,10)
		steeringSlider = p.addUserDebugParameter("steering",-0.5,0.5,0)

		ray_length = 2
		angle_swept = 60
		step = math.ceil(100*angle_swept/p.MAX_RAY_INTERSECTION_BATCH_SIZE)/100
		angles = np.arange(-angle_swept/2, angle_swept/2, step) * np.pi / 180 #angle of rotations
		num_rays = np.shape(angles)[0]
		rays = np.concatenate(([ray_length*np.sin(angles)], [ray_length*np.cos(angles)], [np.zeros(num_rays)]), axis=0)

		rot = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]])
		offset = np.array([0, 0, 0.3])
		count = 0
		nowhere_count = 0
		path = __Path()
		nowhere = False

		while (path.rad < radius + .5):
			# maxForce = p.readUserDebugParameter(maxForceSlider)
			# targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
			# steeringAngle = p.readUserDebugParameter(steeringSlider)
			maxForce = 10.
			targetVelocity = -5
			steeringAngle = 0
			

		#########---------------START Obstacle Avoidance----------########

			position, orientation = p.getBasePositionAndOrientation(car)

			matrix = p.getMatrixFromQuaternion(orientation)
			matrix = np.reshape(matrix, (3, 3))

			src = np.array(position) 
			src = src + np.matmul(matrix,offset)

			path.calcDist(src)
			h = 10

			rays_src = np.repeat([src], num_rays, axis=0)

			orn = np.matmul(matrix, rot) #rotates unit vector y to -x

			rays_end = np.matmul(orn, rays) # unit vector in direction of minitaur
			rays_end = (rays_end + src[:, None]).T
			rays_info = p.rayTestBatch(rays_src.tolist(), rays_end.tolist())

			b = np.asarray([int(i[0]) for i in rays_info])

			for i in range(h-1):

				rays = np.concatenate(([ray_length*np.sin(angles)], [ray_length*np.cos(angles)], [np.full((num_rays,), i+1)]), axis=0)

				rays_end = np.matmul(orn, rays) # unit vector in direction of minitaur
				rays_end = (rays_end + src[:, None]).T

				rays_info = p.rayTestBatch(rays_src.tolist(), rays_end.tolist())

				b = np.vstack((b, np.asarray([int(i[0]) for i in rays_info])))

			nth_ray = find_largest_gap(b)

			if(nth_ray == None):
				nowhere = True
				targetVelocity = 0
				# print("Nowhere")
				nowhere_count += 1
				if nowhere_count > 20:
					break
			else:
				nowhere_count = 0
				deg = 1.*angle_swept*nth_ray/b.shape[1] - angle_swept/2.
				# print("Rotate {:.1f} degrees".format(deg))
				if math.fabs(deg)  > 5:
					targetVelocity = -4
					steeringAngle = np.sign(deg)*.7


		#########---------------END Obstacle Avoidance----------########	

			
			for wheel in wheels:
				p.setJointMotorControl2(car,wheel,p.VELOCITY_CONTROL,targetVelocity=targetVelocity,force=maxForce)
				
			for steer in steering:
				p.setJointMotorControl2(car,steer,p.POSITION_CONTROL,targetPosition=steeringAngle)

			if (useRealTimeSim==0):
				p.stepSimulation()
			time.sleep(0.01)
			count += 1
			
		writer.writerow( (path.len, path.rad, nowhere, blocks, radius) )
		# print("Dist Travelled: {}\n radial: {}\n deadend: {}\n \
		# 	blocks: {}\n radius: {}".format(path.len, path.rad, 
		# 		nowhere, blocks, radius))


	# collision, distance? mag or actual, deadend, time,




		