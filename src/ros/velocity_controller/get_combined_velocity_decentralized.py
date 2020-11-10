from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from init_formations import LEADER_ID
from maintain_formation_decentralized import maintain_formation, CONTROLLED_ZONE

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

def get_obstacle_avoidance_velocity(robot_pose, laser, robot_id):

  u, w = obstacle_avoidance.rule_based(*laser.measurements, robot_id=robot_id)
  
  x = u*np.cos(robot_pose[YAW]) - EPSILON*w*np.sin(robot_pose[YAW])
  y = u*np.sin(robot_pose[YAW]) + EPSILON*w*np.cos(robot_pose[YAW])

  return np.array([x, y])

def get_noise():
  noise = np.random.uniform(low=-1., high=1., size=2)
  noise = normalize(noise)
  return noise

def get_combined_velocity(robot_pose, leader_pose, leader_rrt_velocity, laser, robot_id):

  # Velocities
  rrt_velocity = normalize(leader_rrt_velocity)
  formation_velocity = np.array([0., 0.])
  obstacle_avoidance_velocity = np.array([0., 0.])
  noise = get_noise()

  desired_position = robot_pose[:2]

  if robot_id != LEADER_ID:
    rrt_velocity = np.array([0., 0.])
    formation_velocity, desired_position = maintain_formation(leader_pose, robot_pose, leader_rrt_velocity, robot_id, laser)
    obstacle_avoidance_velocity = get_obstacle_avoidance_velocity(robot_pose, laser, robot_id)

    min_measurement = min(laser.measurements)
    if min < 1.5:
      formation_velocity = leader_rrt_velocity * 0.5

  combined_velocity = weight_velocities(rrt_velocity, formation_velocity, obstacle_avoidance_velocity, noise)

  # Feedback linearization - convert combined_velocities [x,y] [u,w]
  u, w = rrt_navigation.feedback_linearized(pose=robot_pose, velocity=combined_velocity, epsilon=EPSILON)

  return u, w, desired_position

def weighting(velocity, weight):
  return np.array(velocity * weight)

def normalize(vec):
  mag = np.linalg.norm(vec)
  if mag < .01:
    return np.zeros_like(vec)
  else:
    return vec / mag

def weight_velocities(goal_velocity, formation_velocity, obstacle_velocity, noise_velocity):

  goal_w = .2
  formation_w = .5
  static_obs_avoid_w = 0.35
  noise_w = .05
  overall_weight = 1.5
  # currently no robot avoidance in decentralized algorithm as we do not keep all robot poses

  goal = weighting(goal_velocity, goal_w)
  formation = weighting(formation_velocity, formation_w)
  static_obstacle_avoidance = weighting(obstacle_velocity, static_obs_avoid_w)
  noise = weighting(noise_velocity, noise_w)

  objective = goal + formation
  weighted_sum = objective + static_obstacle_avoidance + noise

  if np.linalg.norm(objective) == 0.:
    weighted_sum = np.zeros_like(weighted_sum)

  return weighted_sum * overall_weight
