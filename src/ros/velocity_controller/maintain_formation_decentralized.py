from init_formations import LEADER_ID, FORMATION, SPACING_DIST, SWITCHED_CORRIDOR_FORMATION, INITIAL_YAW, MAP_PARAMS, ALLOW_FORMATION_ROTATION

import numpy as np

X = 0
Y = 1
YAW = 2

ROBOT_RADIUS = 0.105 / 2.
FORMATION_YAW = MAP_PARAMS["FORMATION_YAW"]

# DEAD ZONE (If a robot is within the dead zone of its desired formation postion, it doesnt move)
DEAD_ZONE = 1.5 * ROBOT_RADIUS
# CONTROL_ZONE (if robot is within the controlled zone, velocity towards position linearly increases the further away it is)
CONTROLLED_ZONE = DEAD_ZONE + SPACING_DIST

def get_desired_positions(formation, formation_pose):
  # Take formation and transform (rotate, translate) onto formation_pose
  if ALLOW_FORMATION_ROTATION:
    theta = formation_pose[YAW]
  else:
    theta = FORMATION_YAW - INITIAL_YAW

  desired_positions = np.zeros_like(formation)
  for i in range(len(formation)):
    # Rotate
    desired_positions[i] = np.matmul([[np.cos(theta), -np.sin(theta)],
                                      [np.sin(theta),  np.cos(theta)]], formation[i])
    # Translate
    desired_positions[i] = formation_pose[:2] + desired_positions[i]
  return desired_positions

def detect_corridor(laser):

  sens_inp = np.tanh(laser.leader_measurements)

  # corridor detected if front is big and left and right are small
  if sens_inp[0] > np.tanh(1.) and sens_inp[3] < np.tanh(.8) and sens_inp[4] < np.tanh(.8):
    return SWITCHED_CORRIDOR_FORMATION
  else:
    return FORMATION
    

def maintain_formation(leader_pose, follower_pose, leader_rrt_velocity, robot_id, laser):

  # Account for leader in using id as index
  if robot_id > LEADER_ID:
    robot_id = robot_id - 1

  # Formation orientation is the angle of formation
  formation_orientation = leader_pose[YAW] - INITIAL_YAW
  formation_pose = np.concatenate((leader_pose[:2], [formation_orientation]))

  # Desired positions of each of the follower robots in the formation (see comment above about replacing formation pose with leader...)
  formation = detect_corridor(laser)
  desired_positions = get_desired_positions(formation=formation, formation_pose=formation_pose)

  # velocity directs the follower robots from their current position to their (desired position in the formation)
  follower_position = follower_pose[:2]
  velocity = desired_positions[robot_id] - follower_position

  # update the velocity (the displacement between the current and desired position) depending on the distance

  distance = np.linalg.norm(velocity)

  # If a robot is within accepted radius of formation position, velocity should be 0
  # DEAD ZONE (If a robot is within the dead zone of its desired formation postion, it doesnt move)
  if distance < DEAD_ZONE:
    velocity = np.zeros_like(velocity)
  elif distance < CONTROLLED_ZONE:
    # we do nothing, velocity is directly proportional to distance
    pass
  else:
    # robot is outside the controlled zone, so we normalize the velocity and then multiply
    # by the radius of the control zone.
    velocity = velocity / distance
    velocity = velocity * CONTROLLED_ZONE

  return velocity / CONTROLLED_ZONE, desired_positions[robot_id]
