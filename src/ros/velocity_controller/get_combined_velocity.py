from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from init_formations import LEADER_ID
from maintain_formation import maintain_formation, CONTROLLED_ZONE

import numpy as np
import obstacle_avoidance
import rrt_navigation

# Feedback linearisation epsilon
EPSILON = .2

ROBOT_DISTANCE = .15
ROBOT_EXTRA_DISTANCE = .21

X = 0
Y = 1
YAW = 2

def get_obstacle_avoidance_velocities(robot_poses, lasers):
  xyoa_velocities = []
  for i in range(len(robot_poses)):
    u, w = obstacle_avoidance.braitenberg(robot_poses, i, *(lasers[i].measurements))

    x = u*np.cos(robot_poses[i][YAW]) - EPSILON*w*np.sin(robot_poses[i][YAW])
    y = u*np.sin(robot_poses[i][YAW]) + EPSILON*w*np.cos(robot_poses[i][YAW])

    xyoa_velocities.append(np.array([x,y]))

  return np.array(xyoa_velocities)

def get_noise():
  noise = np.random.uniform(low=-1., high=1., size=[5, 2])
  noise = normalise_velocities(noise)

  return noise

def normalise_velocities(velocities):
  # Accounts for small magnitudes
  for i in range(len(velocities)):
    n = np.linalg.norm(velocities[i])

    if n < 1e-2: 
      velocities[i] = np.zeros_like(velocities[i])
    else:
      velocities[i] = velocities[i] / n
    
  return velocities
      

def get_combined_velocities(robot_poses, leader_rrt_velocity, lasers):

  # get leader and follower poses
  leader_pose = robot_poses[LEADER_ID]
  follower_poses = np.array([robot_poses[i] for i in range(len(robot_poses)) if i != LEADER_ID])

  # Velocities
  follower_formation_velocities, desired_positions, in_dead_zone = maintain_formation(leader_pose, follower_poses, leader_rrt_velocity, lasers)
  obstacle_avoidance_velocities = get_obstacle_avoidance_velocities(robot_poses, lasers)

  # NOTE: for numpy insert, obj is the index of insertion.
  formation_velocities = np.insert(arr=follower_formation_velocities, obj=LEADER_ID, values=np.array([0., 0.]), axis=0)
  # follower formation velocities is only 4 long
  rrt_velocities = np.insert(arr=np.zeros_like(follower_formation_velocities), obj=LEADER_ID, values=leader_rrt_velocity, axis=0)

  # normalized noise and rrt velocities.
  noise_velocities = get_noise()
  rrt_velocities = normalise_velocities(rrt_velocities)

  combined_velocities = weight_velocities(rrt_velocities, formation_velocities, obstacle_avoidance_velocities, robot_avoidance_weights(robot_poses), noise_velocities, in_dead_zone)

  # Feedback linearization - convert combined_velocities [[x,y], ...] into [[u, w], ...]
  us = []
  ws = []
  for i in range(len(combined_velocities)):
    u, w = rrt_navigation.feedback_linearized(pose=robot_poses[i], velocity=combined_velocities[i], epsilon=EPSILON)

    us.append(u)
    ws.append(w)

  return us, ws, desired_positions

def robot_avoidance_weights(robot_poses):
  # now robots stop if there is a robot infront of it.
  # for each robot, if any of the other robot poses are within say pi/2 or pi-delta infront of the robot at a small distance,
  # stop and wait for them to move.
  v = []
  for i in range(len(robot_poses)):
    v.append(1.)
    for j in range(len(robot_poses)):
      # for robots other than this one
      if j != i:
        # if the other robot is infront of it, and distance is < avoidance_threshold
        vector_to_robot = robot_poses[j] - robot_poses[i]
        distance = np.linalg.norm(vector_to_robot[:2])

        angle_to_robot = np.arctan2(vector_to_robot[Y], vector_to_robot[X]) - robot_poses[i][YAW]

        if -np.pi/2. < angle_to_robot < np.pi/2. and distance < ROBOT_EXTRA_DISTANCE:
          # stop dealock (if angle is big, i.e robots are next to each other, let the one with lower id go first.)
          if abs(angle_to_robot) > np.pi/3. and i > j:
            continue
          # if the robots are very close (or quite close but next to each other) stop the lower id robot
          elif distance < ROBOT_DISTANCE or abs(angle_to_robot) > np.pi/3.:
            v[i] = 0.
          break
  return v

def weighting(velocities, weight):
  wv = np.array(velocities)
  for i in range(len(velocities)):
    wv[i] = velocities[i] * weight
  return wv

def weight_velocities(goal_velocities, formation_velocities, obstacle_velocities, robot_avoidance_weights, noise_velocities, in_dead_zone):

  goal_w = .35
  formation_w = .2
  static_obs_avoid_w = .22
  noise_w = .05
  overall_weight = 1.5

  goal = weighting(goal_velocities, goal_w)
  formation = weighting(formation_velocities, formation_w)
  static_obstacle_avoidance = weighting(obstacle_velocities, static_obs_avoid_w)
  noise = weighting(noise_velocities, noise_w)

  # only leader has the goal, the rest have formation constraints
  objective = goal + formation
          
  # sum of all velocity components
  weighted_sum = objective + static_obstacle_avoidance + noise

  # RULE: For each robot, if it is nearer other robots, let the first robot (by id) pass
  for i in range(len(objective)):
    if robot_avoidance_weights[i] == 0.:
      weighted_sum[i] = np.zeros_like(weighted_sum[i])

  # RULE: For each robot, if it has reached its objective, stop (ignore obstacle input)
  for i in range(len(objective)):
    if np.linalg.norm(objective[i]) == 0.:
      weighted_sum[i] = np.zeros_like(weighted_sum[i])

  # RULE: if the robots are outside the deadzone, the leader has to slow down
  not_in_dead_zone = len(in_dead_zone) - sum(in_dead_zone)
  if not_in_dead_zone > 0:
    weighted_sum[LEADER_ID] = weighted_sum[LEADER_ID] * .3

  return weighted_sum * overall_weight
