#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import matplotlib.pyplot as plt
import numpy as np
import os
import random
import re
import rospy
import sys
import yaml

directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../python')
sys.path.insert(0, directory)
import rrt

directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '/velocity_controller')
sys.path.insert(0, directory)
from init_formations import FORMATION, LEADER_ID, MAP_PARAMS, RUN_RRT
import get_combined_velocity as gcv
import rrt_navigation


# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
# Laser scan message:
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
from sensor_msgs.msg import LaserScan
# For groundtruth information.
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

ROBOT_NAMES = ["tb3_0", "tb3_1", "tb3_2", "tb3_3", "tb3_4"]

GOAL_POSITION = MAP_PARAMS["GOAL_POSITION"]

EPSILON = .2

MAP = MAP_PARAMS["MAP_NAME"]

ERRORS = [[] for _ in range(len(ROBOT_NAMES)-1)]

X = 0
Y = 1
YAW = 2

FREE = 0
UNKNOWN = 1
OCCUPIED = 2

class SimpleLaser(object):
  def __init__(self, name):
    rospy.Subscriber('/' + name + '/scan', LaserScan, self.callback)
    self._angles = [0., np.pi / 4., -np.pi / 4., np.pi / 2., -np.pi / 2.]
    self._width = np.pi / 180. * 10.  # 10 degrees cone of view.
    self._measurements = [float('inf')] * len(self._angles)
    self._indices = None

  def callback(self, msg):
    # Helper for angles.
    def _within(x, a, b):
      pi2 = np.pi * 2.
      x %= pi2
      a %= pi2
      b %= pi2
      if a < b:
        return a <= x and x <= b
      return a <= x or x <= b;

    # Compute indices the first time.
    if self._indices is None:
      self._indices = [[] for _ in range(len(self._angles))]
      for i, d in enumerate(msg.ranges):
        angle = msg.angle_min + i * msg.angle_increment
        for j, center_angle in enumerate(self._angles):
          if _within(angle, center_angle - self._width / 2., center_angle + self._width / 2.):
            self._indices[j].append(i)

    ranges = np.array(msg.ranges)
    for i, idx in enumerate(self._indices):
      # We do not take the minimum range of the cone but the 10-th percentile for robustness.
      self._measurements[i] = np.percentile(ranges[idx], 10)

  @property
  def ready(self):
    return not np.isnan(self._measurements[0])

  @property
  def measurements(self):
    return self._measurements


class GroundtruthPose(object):
  def __init__(self, name='turtlebot3_burger'):
    rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
    self._pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
    self._name = name

  def callback(self, msg):
    idx = [i for i, n in enumerate(msg.name) if n == self._name]
    if not idx:
      raise ValueError('Specified name "{}" does not exist.'.format(self._name))
    idx = idx[0]
    self._pose[0] = msg.pose[idx].position.x
    self._pose[1] = msg.pose[idx].position.y
    _, _, yaw = euler_from_quaternion([
        msg.pose[idx].orientation.x,
        msg.pose[idx].orientation.y,
        msg.pose[idx].orientation.z,
        msg.pose[idx].orientation.w])
    self._pose[2] = yaw

  @property
  def ready(self):
    return not np.isnan(self._pose[0])

  @property
  def pose(self):
    return self._pose
  
def save_errors(robot_poses, desired_positions):
  follower_positions = np.array([robot_poses[i][:2] for i in range(len(robot_poses)) if i != LEADER_ID])
  pose_err = [np.linalg.norm(f_pos - d_pos) for f_pos, d_pos in zip(follower_positions, desired_positions)]

  for i in range(len(pose_err)):
    ERRORS[i].append(pose_err[i])

def plot_errors():
  sampled_errs = [ERRORS[i][::20] for i in range(len(ERRORS))]

  x = np.arange(len(sampled_errs[0]))
  for i in range(len(sampled_errs)):
    plt.plot(x, sampled_errs[i])

  plt.xlabel('Time')
  plt.ylabel('Error')
  plt.legend([ROBOT_NAMES[i] for i in range(len(ROBOT_NAMES)) if i != LEADER_ID])
  plt.show()

def run():
  rospy.init_node('obstacle_avoidance')

  # Update control every 50 ms.
  rate_limiter = rospy.Rate(50)
  
  publishers = [None] * len(ROBOT_NAMES)
  lasers = [None] * len(ROBOT_NAMES)
  groundtruths = [None] * len(ROBOT_NAMES)

  # RRT path
  # If RUN_RRT is False, load the predefined path
  if not RUN_RRT:
    current_path = MAP_PARAMS["RRT_PATH"]
  else:
    current_path = None
  
  vel_msgs = [None] * len(ROBOT_NAMES)
  for i,name in enumerate(ROBOT_NAMES):
    publishers[i] = rospy.Publisher('/' + name + '/cmd_vel', Twist, queue_size=5)
    lasers[i] = SimpleLaser(name)
    groundtruths[i] = GroundtruthPose(name)

  # Load map. (in here so it is only computed once)
  with open(os.path.expanduser('~/catkin_ws/src/exercises/project/python/{}.yaml'.format(MAP))) as fp:
    data = yaml.load(fp)
  img = rrt.read_pgm(os.path.expanduser('~/catkin_ws/src/exercises/project/python/{}.pgm'.format(MAP)), data['image'])
  occupancy_grid = np.empty_like(img, dtype=np.int8)
  occupancy_grid[:] = UNKNOWN
  occupancy_grid[img < .1] = OCCUPIED
  occupancy_grid[img > .9] = FREE
  # Transpose (undo ROS processing).
  occupancy_grid = occupancy_grid.T
  # Invert Y-axis.
  occupancy_grid = occupancy_grid[:, ::-1]
  occupancy_grid = rrt.OccupancyGrid(occupancy_grid, data['origin'], data['resolution'])

  while not rospy.is_shutdown():
    # Make sure all measurements are ready.
    if not all(laser.ready for laser in lasers) or not all(groundtruth.ready for groundtruth in groundtruths):
      rate_limiter.sleep()
      continue

    # Compute RRT for the leader only
    while not current_path:
        start_node, final_node = rrt.rrt(groundtruths[LEADER_ID].pose, GOAL_POSITION, occupancy_grid)

        current_path = rrt_navigation.get_path(final_node)
        # print("CURRENT PATH: ", current_path)

    # get the RRT velocity for the leader robot
    position = np.array([groundtruths[LEADER_ID].pose[X] + EPSILON*np.cos(groundtruths[LEADER_ID].pose[YAW]),
                         groundtruths[LEADER_ID].pose[Y] + EPSILON*np.sin(groundtruths[LEADER_ID].pose[YAW])], dtype=np.float32)
    rrt_velocity = rrt_navigation.get_velocity(position, np.array(current_path, dtype=np.float32))

    # get robot poses
    robot_poses = np.array([groundtruths[i].pose for i in range(len(groundtruths))])

    # get the velocities for all the robots
    us, ws, desired_positions = gcv.get_combined_velocities(robot_poses=robot_poses, leader_rrt_velocity=rrt_velocity, lasers=lasers)

    save_errors(robot_poses, desired_positions)

    for i in range(len(us)):
      vel_msgs[i] = Twist()
      vel_msgs[i].linear.x = us[i]
      vel_msgs[i].angular.z = ws[i]

    for i,_ in enumerate(ROBOT_NAMES):
      publishers[i].publish(vel_msgs[i])

    rate_limiter.sleep()


if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException:
    pass
  plot_errors()
