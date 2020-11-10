from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from rrt_navigation import feedback_linearized
from init_formations import INITIAL_YAW

import numpy as np

SENSOR_ROBOT_TOLERANCE = 0.08
ROBOT_DETECTION_THRESHOLD = 3.5
RIGHT, FRONT_RIGHT, FRONT, FRONT_LEFT, LEFT = 0, 1, 2, 3, 4
MAX_DISTANCE = 3.5

X = 0
Y = 1
YAW = 2

def braitenberg(robot_poses, robot_id, front, front_left, front_right, left, right):
  u = .1  # [m/s] so when not facing an obstacle the robot accelerates
  w = 0.  # [rad/s] going counter-clockwise.
  
  # create an array of sensor input from the robot
  sens_inp = np.array([right, front_right, front, front_left, left])

  # detect the other robots within the sensor range
  sens_inp = detect_robot_presence(robot_poses, sens_inp, robot_id)
  
  # smooth with arctan (could use tanh), but then take the gradient of this, so big inputs give little change, but small imputs give the most change
  # this is the gradient of the tanh function
  sens_inp = 1. / (1. + 3.*(sens_inp**2.))

  # rearrange for braitenburg
  sens_inp = np.array([sens_inp[FRONT],sens_inp[FRONT_LEFT],sens_inp[FRONT_RIGHT],sens_inp[LEFT],sens_inp[RIGHT]])

  # Direction to turn from front sensor
  # DIRECTION causes the SPLIT AND MERGE behaviour. Robots on LHS of formation go left, robots on RHS of formation go right.
  if robot_id < len(robot_poses) // 2:
    direction =  1.
  else:
    direction = -1.

  # this should be a better form of weights, when it is surrounded, acceleration is negative default
  # a bias on the front sensor to steer slighty left is given so that it does not get stuck still in reverse (it can manage to turn around, test this with extra objects)
  # actually the plus on left and right makes sense, i want it to get smaller when it gets bigger
  weights = np.array([
    # front front_left front_right left right
    [-.3,          -.2, -.2, 0., 0.],
    [.5*direction, -5.,  5., .3, .3]
  ])

  # multiply sense inputs by the weights.
  params = np.matmul(weights, sens_inp)

  # add a function to turn around the robot when it gets very very close
  # prevents the non turning issue when it goes head on into something
  if front < .5 and front_left > .2 and front_right > .2:
    w = w + (40. * (1. / (1. + 100.*(front**2.))) * direction)
  
  # extract params note this doesn't consider previous step since we set them to 0 / 0.5 at the start.. it does do what you think nice.
  u, w = u + params[0], w + params[1]

  # CORRIDOR function, this allows the robot to move through narrow corridors without obstacle avoidance slowing it down
  # NOTE: the signs and values are the opposite of what you would expect for tanh , since this uses a different smoothing function (1/3x^2)
  if np.tanh(front) < .14 and np.tanh(front_left) < .869 and np.tanh(front_right) < .869:
    # print("CORRIDOR RULE TRIGGERED")
    w = - .5*(1.-sens_inp[FRONT_LEFT]) + .5*(1.-sens_inp[FRONT_RIGHT])
    u = .1

  return u, w

def rule_based(front, front_left, front_right, left, right, robot_id):
  u = 0.  # [m/s]
  w = 0.  # [rad/s] going counter-clockwise.

  # apply tanh to the input to deal with infinity
  sens_inp = np.array([front, front_left, front_right, left, right])
  sens_inp = np.tanh(sens_inp)

 # if the right hand side detects an approaching object , alter w to move left
  if sens_inp[2] < np.tanh(.4):
    # print("fire front left")
    u -= 0.05
    w = w + 1.5*(1.3-sens_inp[2])

  # if the left hand side detects and approaching object, alter w to move to the right
  if sens_inp[1] < np.tanh(.4):
    u -= 0.05
    # print("fire front right")
    w = w - 1.5*(1.3-sens_inp[1])

   # if robot is very close to the right hand slide, adjust left a little
  if sens_inp[4] < np.tanh(.2):
    # print("fire left")
    w = w + 0.3*(1.-sens_inp[4])

  # if robot is very close to the left hand slide, adjust right a little
  if sens_inp[3] < np.tanh(.2):
    # print("fire right")
    w = w - 0.3*(1.-sens_inp[3])

  direction = 1. # if robot_id > 2 else -1.


  # if close to front, move away
  if sens_inp[0] < np.tanh(.6):
    # print("FRONT firing")
    u = -0.2
    # if sens_inp[1] < sens_inp[2]:
    #   w =  w -2.5*direction*(1-0.5*sens_inp[0])
    # else:
    #   w = w +2.5*direction*(1-0.5*sens_inp[0])
    w = w +4.*direction*(1-0.5*sens_inp[0])

  # print("u, w: ", u, w)

  return u, w

  # if all sensors detect obstacles are far away, adjust slightly using a highest absolute value for direction
  if (sens_inp > np.tanh(.6)).all():
    # move in coward motion, away from the closest object
    return u, sens_inp[2] + sens_inp[4] - sens_inp[1] - sens_inp[3]

  # breakout! (mentioned in part C only - rest of rules are as described in part a) 
  # If the front sensor reports a much greater distance than the side sensors, 
  # don't change direction so much, use the left and right sensors for small adjustments
  if sens_inp[0] > np.tanh(.6) and sens_inp[0] > 2.*sens_inp[1] and sens_inp[0] > 2.*sens_inp[2]:
    # override w updates and steer using left and right sensors, which should be close enough to detect collisions
    # in tight areas
    if sens_inp[3] < np.tanh(.3):
      w = w + 1.5*(1.-sens_inp[3])
    if sens_inp[4] < np.tanh(.3):
      w = w - 1.5*(1.-sens_inp[25])
    if sens_inp[1] < np.tanh(.3):
      w = w + 1.5*(1.-sens_inp[1])
    if sens_inp[2] < np.tanh(.3):
      w = w - 1.5*(1.-sens_inp[2])

    return u, w

  # if the right hand side detects an approaching object , alter w to move left
  if sens_inp[2] < np.tanh(.3):
    w = w + 4.*(1.-sens_inp[2])

  # if the left hand side detects and approaching object, alter w to move to the right
  if sens_inp[1] < np.tanh(.3):
    w = w - 4.*(1.-sens_inp[1])

  # if robot is very close to the right hand slide, adjust left a little
  if sens_inp[4] < np.tanh(.3):
    w = w + 2.*(1.-sens_inp[4])

  # if robot is very close to the left hand slide, adjust right a little
  if sens_inp[3] < np.tanh(.3):
    w = w - 2.*(1.-sens_inp[3])
  
  return u, w


def detect_robot_presence(robot_poses, raw_sensor_input, robot_id):
  # CHECK IF SENSORS ARE BEING TRIGGERED BY OTHER ROBOTS
  # compute distances to other robots
  robot_pose = robot_poses[robot_id]
  distances = [np.linalg.norm(robot_pose - r_p) for r_p in robot_poses]

  # vectors from current robot to all the other robots
  vectors = [r_p[:2] - robot_pose[:2] for r_p in robot_poses]

  sensor_angles = [-np.pi/2., -np.pi/4., 0., np.pi/4, np.pi/2.]

  # for each sensor, if there is a robot infront of it, ignore it (by setting the distance it measures to max distance)
  for i in range(len(sensor_angles)):
    if robot_infront_of_sensor(sensor_angles[i], raw_sensor_input[i], robot_poses, distances, vectors, robot_id):
      raw_sensor_input[i] = MAX_DISTANCE

  return raw_sensor_input

def robot_infront_of_sensor(sensor_angle, raw_sensor_value, robot_poses, robot_distances, robot_displacement_vectors, robot_id):
  # check if any of the other robots are infront of the sensor (within a distance of ROBOT_DETECTION_THRESHOLD)
  
  robot_pose = robot_poses[robot_id]
  found_robot = False

  for i in range(len(robot_poses)):
    if i != robot_id:
      # if robot within range and sensor_value is similar to robot_distance[i]
      if robot_distances[i] < ROBOT_DETECTION_THRESHOLD and abs(raw_sensor_value - robot_distances[i]) < SENSOR_ROBOT_TOLERANCE:
        # if the angle between the two robots (minus the current robot yaw) is within +- pi/8, there is a robot infront of the sensor
        angle_to_robot = np.arctan2(robot_displacement_vectors[i][Y], robot_displacement_vectors[i][X])
        angle_to_robot -= robot_pose[YAW]

        # pi/8. is used since the sensors are pi/4. apart on the robot
        if abs(angle_to_robot - sensor_angle) < np.pi/8.:
          found_robot = True
          break
  
  return found_robot
