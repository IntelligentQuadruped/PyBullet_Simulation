from minitaur import Minitaur
import pybullet as p
import pybullet_data
import numpy as np
import time
import sys
import math
import itertools
import operator

minitaur = None
halfpi = math.pi/2

evaluate_func_map = dict()

go_straight = [-1,-1,-1,-1,1,1,1,1]
go_back = [1,1,1,1,-1,-1,-1,-1]
go_left = [-1,-1,-1,-1,-1,-1,-1,-1]
go_right = [1,1,1,1,1,1,1,1]

def current_position():
  global minitaur
  position = minitaur.getBasePosition()
  return np.asarray(position)


def is_fallen():
  global minitaur
  orientation = minitaur.getBaseOrientation()
  rotMat = p.getMatrixFromQuaternion(orientation)
  localUp = rotMat[6:]
  return np.dot(np.asarray([0, 0, 1]), np.asarray(localUp)) < 0

def evaluate_desired_motorAngle_8Amplitude8Phase(i, params):
  nMotors = 8
  speed = 0.35
  for jthMotor in range(nMotors):
    joint_values[jthMotor] = math.sin(i*speed + params[nMotors + jthMotor])*params[jthMotor]*+ halfpi
  return joint_values

def evaluate_desired_motorAngle_2Amplitude4Phase(i, params):
  speed = 0.35
  phaseDiff = params[2]
  a0 = math.sin(i * speed) * params[0] + halfpi
  a1 = math.sin(i * speed + phaseDiff) * params[1] + halfpi
  a2 = math.sin(i * speed + params[3]) * params[0] +  halfpi
  a3 = math.sin(i * speed + params[3] + phaseDiff) * params[1] +  halfpi
  a4 = math.sin(i * speed + params[4] + phaseDiff) * params[1] +  halfpi
  a5 = math.sin(i * speed + params[4]) * params[0] +  halfpi
  a6 = math.sin(i * speed + params[5] + phaseDiff) * params[1] +  halfpi
  a7 = math.sin(i * speed + params[5]) * params[0] +  halfpi
  joint_values = [a0, a1, a2, a3, a4, a5, a6, a7]
  return joint_values

def evaluate_desired_motorAngle_hop(i, params):
  amplitude = params[0]
  speed = params[1]
  a1 = math.sin(i*speed)*amplitude+ halfpi
  a2 = math.sin(i*speed+math.pi)*amplitude+ halfpi
  joint_values = [a1,  halfpi, a2,  halfpi,  halfpi, a1,  halfpi, a2]
  return joint_values


evaluate_func_map['evaluate_desired_motorAngle_8Amplitude8Phase'] = evaluate_desired_motorAngle_8Amplitude8Phase
evaluate_func_map['evaluate_desired_motorAngle_2Amplitude4Phase'] = evaluate_desired_motorAngle_2Amplitude4Phase
evaluate_func_map['evaluate_desired_motorAngle_hop'] = evaluate_desired_motorAngle_hop




#########---------------START Obstacle Avoidance----------########

def find_largest_gap(collisions):
  gaps = (list(i) for (val,i) in itertools.groupby((enumerate(collisions)),operator.itemgetter(1)) if val == -1)
  max_gap = max(gaps, key=len)
  return math.floor((max_gap[-1][0]-max_gap[0][0])/2)


#########---------------END Obstacle Avoidance----------########






def evaluate_params(evaluateFunc, params, objectiveParams, urdfRoot='', timeStep=0.01, maxNumSteps=10000, sleepTime=0):
  print('start evaluation')
  beforeTime = time.time()
  p.resetSimulation()

  p.setTimeStep(timeStep)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())

  p.loadURDF("%s/plane.urdf" % urdfRoot)
  p.setGravity(0,0,-9.81)

  mass = 1
  visualShapeId = -1
  cube =p.createCollisionShape(p.GEOM_MESH,fileName="cube.obj",flags=p.GEOM_FORCE_CONCAVE_TRIMESH, meshScale=[1,1,1])
  orn = p.getQuaternionFromEuler([0,0,0])
  print(orn)
  p.createMultiBody (0,cube, baseOrientation=orn, basePosition=[-1,1,0])
  p.createMultiBody (0,cube, baseOrientation=orn, basePosition=[-2,2,0])
  p.createMultiBody (0,cube, baseOrientation=orn, basePosition=[-1,-1,0])



  global minitaur
  minitaur = Minitaur(urdfRoot)
  start_position = current_position()
  last_position = None  # for tracing line
  total_energy = 0



#########---------------START Obstacle Avoidance----------########

  ray_length = 1
  angle_swept = 130
  step = math.ceil(100*angle_swept/p.MAX_RAY_INTERSECTION_BATCH_SIZE)/100
  angles = np.arange(-angle_swept/2, angle_swept/2, step) * np.pi / 180 #angle of rotations
  num_angles = np.shape(angles)[0]
  rays = np.concatenate(([ray_length*np.sin(angles)], [ray_length*np.cos(angles)], [np.zeros(num_angles)]), axis=0)
  rot = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]])
  offset = np.array([-0.33, 0, 0.07])


  for i in range(maxNumSteps):
    matrix = p.getMatrixFromQuaternion(minitaur.getBaseOrientation())
    matrix = np.reshape(matrix, (3, 3))

    src = np.array(minitaur.getBasePosition()) 
    src = src + np.matmul(matrix,offset)

    rays_src = np.repeat([src], num_angles, axis=0)

    orn = np.matmul(matrix, rot) #rotates unit vector y to -x
    rays_end = np.matmul(orn, rays) # unit vector in direction of minitaur

    rays_end = (rays_end + src[:, None]).T
    rays_info = p.rayTestBatch(rays_src.tolist(), rays_end.tolist())

    b = [int(i[0]) for i in rays_info]
    nth_ray = find_largest_gap(b)
    print("Rotate {:.1f}degrees".format(angles[nth_ray]*180/np.pi))


#########---------------END Obstacle Avoidance----------########





    torques = minitaur.getMotorTorques()
    velocities = minitaur.getMotorVelocities()
    total_energy += np.dot(np.fabs(torques), np.fabs(velocities)) * timeStep

    joint_values = evaluate_func_map[evaluateFunc](i, params)

    minitaur.applyAction(joint_values, go_straight)
    p.stepSimulation()
    if (is_fallen()):
      break





    if i % 100 == 0:
      sys.stdout.write('.')
      sys.stdout.flush()
    time.sleep(sleepTime)

  print(' ')

  alpha = objectiveParams[0]
  final_distance = np.linalg.norm(start_position - current_position())
  finalReturn = final_distance - alpha * total_energy
  elapsedTime = time.time() - beforeTime
  print ("trial for ", params, " final_distance", final_distance, "total_energy", total_energy, "finalReturn", finalReturn, "elapsed_time", elapsedTime)
  return finalReturn





