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

def detect_corridor(robot_poses, lasers):
  # it's a corridor if the majority think it is a corridor, or if the leader thinks it's a corridor
  # this approximates waiting until the half the robots leave the corridor to switch back to the normal formation

  believe_in_corridor = []
  for i in range(len(robot_poses)):
    sens_inp = np.tanh(lasers[i].measurements)

    # corridor detected if front is big and left and right are small
    if sens_inp[0] > np.tanh(1.) and sens_inp[3] < np.tanh(.8) and sens_inp[4] < np.tanh(.8):
      believe_in_corridor.append(1.)
    else:
      believe_in_corridor.append(0.)

  total_in_corridor = np.sum(np.array(believe_in_corridor))

  if believe_in_corridor[LEADER_ID] == 1. or total_in_corridor > len(robot_poses)/2.:
    return SWITCHED_CORRIDOR_FORMATION
  else:
    return FORMATION

def maintain_formation(leader_pose, follower_poses, leader_rrt_velocity, lasers):
  # Formation orientation is the angle of the formation given the leader's direction.
  formation_orientation = leader_pose[YAW] - INITIAL_YAW
  formation_pose = np.concatenate((leader_pose[:2], [formation_orientation]))

  # Desired positions of each of the follower robots in the formation
  robot_poses = np.insert(arr=follower_poses, obj=LEADER_ID, values=leader_pose, axis=0)
  formation = detect_corridor(robot_poses, lasers)
  desired_positions = get_desired_positions(formation=formation, formation_pose=formation_pose)

  # velocity directs the follower robots from their current position to their (desired position in the formation)
  follower_positions = np.array([pose[:2] for pose in follower_poses])
  velocities = desired_positions - follower_positions

  distances = []

  # update each velocity (the displacement between the current and desired position) depending on the distance
  for i in range(len(velocities)):

    distance = np.linalg.norm(velocities[i])
    distances.append(distance)

    # If a robot is within accepted radius of formation position, velocity should be 0
    # DEAD ZONE (If a robot is within the dead zone of its desired formation postion, it doesnt move)
    if distance < DEAD_ZONE:
      velocities[i] = np.zeros_like(velocities[i])
    elif distance < CONTROLLED_ZONE:
      # we do nothing, velocity is directly proportional to distance
      pass
    else:
      # robot is outside the controlled zone, so we normalize the velocity and then multiply
      # by the radius of the control zone.
      velocities[i] = velocities[i] / distance
      velocities[i] = velocities[i] * CONTROLLED_ZONE

  # scale to be between 0 and 1
  velocities = velocities / CONTROLLED_ZONE

  # in dead zone vector
  distances = np.array(distances)
  in_dead_zone = distances < DEAD_ZONE * 3.

  return velocities, desired_positions, in_dead_zone
