from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import rrt

SPEED = .5

X = 0
Y = 1
YAW = 2


def feedback_linearized(pose, velocity, epsilon):
  u = 0.  # [m/s]
  w = 0.  # [rad/s] going counter-clockwise.

  # check if velocity is near zero:
  threshold = 0.01
  if np.linalg.norm(velocity) < threshold:
    return u, w 

  theta = pose[YAW]

  u =  velocity[X]*np.cos(theta) + velocity[Y]*np.sin(theta)
  w = (velocity[Y]*np.cos(theta) - velocity[X]*np.sin(theta)) / epsilon

  return u, w


def get_velocity(position, path_points):
  v = np.zeros_like(position)
  if len(path_points) == 0:
    return v
  # Stop moving if the goal is reached.
  if np.linalg.norm(position - path_points[-1]) < .2:
    return v

  minp = path_points[0]
  mind = np.linalg.norm(path_points[0]-position)
  nextp = path_points[1]

  for u, v in zip(path_points[1:], path_points[2:]):
    if np.linalg.norm(u-position) < mind:
      minp = u
      mind = np.linalg.norm(u-position)
      nextp = v

  vec = nextp - position
  vec = vec / np.linalg.norm(vec)

  return SPEED * vec
  

def get_path(final_node):
  # Construct path from RRT solution.
  if final_node is None:
    return []
  path_reversed = []
  path_reversed.append(final_node)
  while path_reversed[-1].parent is not None:
    path_reversed.append(path_reversed[-1].parent)
  path = list(reversed(path_reversed))
  # Put a point every 5 cm.
  distance = 0.05
  offset = 0.
  points_x = []
  points_y = []
  for u, v in zip(path, path[1:]):
    center, radius = rrt.find_circle(u, v)
    du = u.position - center
    theta1 = np.arctan2(du[1], du[0])
    dv = v.position - center
    theta2 = np.arctan2(dv[1], dv[0])
    # Check if the arc goes clockwise.
    clockwise = np.cross(u.direction, du).item() > 0.
    # Generate a point every 5cm apart.
    da = distance / radius
    offset_a = offset / radius
    if clockwise:
      da = -da
      offset_a = -offset_a
      if theta2 > theta1:
        theta2 -= 2. * np.pi
    else:
      if theta2 < theta1:
        theta2 += 2. * np.pi
    angles = np.arange(theta1 + offset_a, theta2, da)
    offset = distance - (theta2 - angles[-1]) * radius
    points_x.extend(center[X] + np.cos(angles) * radius)
    points_y.extend(center[Y] + np.sin(angles) * radius)
  return zip(points_x, points_y)
