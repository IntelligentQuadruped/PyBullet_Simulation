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
			p.addUserDebugLine(prev, pos, [0,0,0])
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


class __Rays:
	def __init__(self, ray_length=2, h=10):
		self.ray_length = ray_length
		self.angle_swept = 60
		step = math.ceil(100*self.angle_swept/p.MAX_RAY_INTERSECTION_BATCH_SIZE)/100
		self.angles = np.arange(-self.angle_swept/2, self.angle_swept/2, step) * np.pi / 180 #angle of rotations
		self.num_rays = np.shape(self.angles)[0]
		self.rays = np.concatenate(([ray_length*np.sin(self.angles)], [ray_length*np.cos(self.angles)], [np.zeros(self.num_rays)]), axis=0)
		self.rot = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]])
		self.offset = np.array([0, 0, 0.3])
		self.h = h


	def get_rays_data(self, position, orientation, turn=None):
		ray_length = self.ray_length
		src = np.array(position)
		matrix = np.reshape(p.getMatrixFromQuaternion(orientation), (3, 3))
		src = src + np.matmul(matrix, self.offset)

		rays_src = np.repeat([src], self.num_rays, axis=0)

		orn = np.matmul(matrix, self.rot) #rotates unit vector y to -x
		if turn is not None:
			orn = np.matmul(orn, turn)

		rays_end = np.matmul(orn, self.rays) # unit vector in direction of minitaur
		rays_end = (rays_end + src[:, None]).T

		rays_info = p.rayTestBatch(rays_src.tolist(), rays_end.tolist())

		b = np.asarray([int(i[0]) for i in rays_info])

		for i in range(self.h - 1):

			rays_h = np.concatenate(([ray_length*np.sin(self.angles)], [ray_length*np.cos(self.angles)], [np.full((self.num_rays,), i+1)]), axis=0)

			rays_end = np.matmul(orn, rays_h) # unit vector in direction of minitaur
			rays_end = (rays_end + src[:, None]).T

			rays_info = p.rayTestBatch(rays_src.tolist(), rays_end.tolist())

			b = np.vstack((b, np.asarray([int(i[0]) for i in rays_info])))

		return src, b

	def get_orn(self, orientation, turn):
		matrix = np.reshape(p.getMatrixFromQuaternion(orientation), (3, 3))
		orn = np.matmul(matrix, self.rot)
		a = np.linalg.norm(orn-turn) 
		print(orn.flatten())
		print(turn.flatten())
		print(a)
		return a > 0.1


	def turn_head(self, position, orientation):
		x = 90
		rang = np.arange(-x, x, 10)
		for deg in rang:
			theta = deg*np.pi/180
			cos = np.cos(theta)
			sin = np.sin(theta)
			rotation = np.array([[cos, -sin, 0], [sin, cos, 0], [0, 0, 0]])
			_, b = self.get_rays_data(position, orientation, rotation)
			nth_ray = find_largest_gap(b)

			if(nth_ray == None):
				continue
			else:
				angle_swept = self.angle_swept
				nth = 1.*angle_swept*nth_ray/b.shape[1] - angle_swept/2.
				cos1 = np.cos((deg+nth)*np.pi/180)
				sin1 = np.sin((deg+nth)*np.pi/180)
				turn = np.array([[cos1, -sin1, 0], [sin1, cos1, 0], [0, 0, 0]])
				return deg + nth, turn
		return None, None


with open('rc_output.csv', 'w+') as f:
	writer = csv.writer(f, quoting=csv.QUOTE_NONNUMERIC)
	writer.writerow( ('Distance Travelled', 'Radial Distance', 'Deadend', 'Blocks', 'Radius', 'Timeout') )

	while True:

		cid = p.connect(p.SHARED_MEMORY)
		if (cid<0):
			p.connect(p.GUI)
			# p.connect(p.DIRECT)

		p.setPhysicsEngineParameter(numSolverIterations=5, fixedTimeStep=1., 
									numSubSteps=50)
			
		p.resetSimulation()
		p.setGravity(0,0,-10)

		useRealTimeSim = False

		cube =p.createCollisionShape(p.GEOM_MESH,fileName=os.path.join(pybullet_data.getDataPath(),"cube.obj"),flags=p.GEOM_FORCE_CONCAVE_TRIMESH, meshScale=[1,1,1])
		orn = p.getQuaternionFromEuler([0,0,0])

		blocks, radius = 125, 10
		random_environment(blocks, radius)

		p.setRealTimeSimulation(useRealTimeSim) # either this
		p.loadURDF(os.path.join(pybullet_data.getDataPath(),"plane.urdf"))

		car = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"racecar/racecar.urdf"))

		inactive_wheels = [5,7]
		wheels = [2,3]
		for wheel in inactive_wheels:
				p.setJointMotorControl2(car,wheel,p.VELOCITY_CONTROL,targetVelocity=0,force=0)
		steering = [4,6]

		rays = __Rays()

		path = __Path()
		nowhere = False
		count = 0
		timout = False

		while (path.rad < radius + .5):

			maxForce, targetVelocity, steeringAngle = 10., -3, 0
			
			pos, orn = p.getBasePositionAndOrientation(car)
			src, b = rays.get_rays_data(pos, orn)

			path.calcDist(src)

			nth_ray = find_largest_gap(b)

			if(nth_ray == None):
				print("Looking by turning head")
				deg, turn = rays.turn_head(pos, orn)
				if turn is None:
					nowhere = True
					break
				else:
					print("Turning head {} degrees".format(deg))
					matrix = np.reshape(p.getMatrixFromQuaternion(orn), (3, 3))
					orn = np.matmul(matrix, rays.rot)
					turn = np.matmul(orn, turn)
					max_a = np.inf
					while(True):
						print("Turning body to {} degrees".format(deg))
						pos, orient = p.getBasePositionAndOrientation(car)
						src, b = rays.get_rays_data(pos, orient)
						path.calcDist(src)
						a = rays.get_orn(orient, turn)
						
						if a < max_a and a > 0.05:
							max_a = a
							targetVelocity = -2
							steeringAngle = np.sign(deg)

							for wheel in wheels:
								p.setJointMotorControl2(car,wheel,p.VELOCITY_CONTROL,targetVelocity=targetVelocity,force=maxForce)
								
							for steer in steering:
								p.setJointMotorControl2(car,steer,p.POSITION_CONTROL,targetPosition=steeringAngle)

							if (useRealTimeSim==0):
								p.stepSimulation()
							time.sleep(0.1)
						else:
							print("Done turning")
							break

			else:
				print("Walking")
				angle_swept = rays.angle_swept
				deg = 1.*angle_swept*nth_ray/b.shape[1] - angle_swept/2.
				# print("Rotate {:.1f} degrees".format(deg))
				if math.fabs(deg)  > 5:
					targetVelocity = -3
					steeringAngle = np.sign(deg)

			for wheel in wheels:
				p.setJointMotorControl2(car,wheel,p.VELOCITY_CONTROL,targetVelocity=targetVelocity,force=maxForce)
				
			for steer in steering:
				p.setJointMotorControl2(car,steer,p.POSITION_CONTROL,targetPosition=steeringAngle)

			if (useRealTimeSim==0):
				p.stepSimulation()
			time.sleep(0.1)
			count+=1
			if (count > 1000):
				timeout = True
			
		# writer.writerow( (path.len, path.rad, nowhere, blocks, radius, timeout) )
		print("Dist Travelled: {}\n radial: {}\n deadend: {}\n \
			blocks: {}\n radius: {} timemout: {}".format(path.len, path.rad, 
				nowhere, blocks, radius, timeout))
		p.disconnect()





		