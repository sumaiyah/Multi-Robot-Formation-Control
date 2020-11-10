#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
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
import get_combined_velocity_decentralized as gcv
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

# This robot's information, default to robot 0
ROBOT_ID = 0
ROBOT_NAME = ROBOT_NAMES[ROBOT_ID]

# Belief of leader
LEADER_NAME = ROBOT_NAMES[LEADER_ID]
LEADER_POSE = [None] * 3
LEADER_VELOCITY = [None] * 2

ERRORS = []

GOAL_POSITION = MAP_PARAMS["GOAL_POSITION"]

EPSILON = .2

MAP = MAP_PARAMS["MAP_NAME"]

X = 0
Y = 1
YAW = 2

FREE = 0
UNKNOWN = 1
OCCUPIED = 2

class SimpleLaser(object):
  def __init__(self, name):
    rospy.Subscriber('/' + name + '/scan', LaserScan, self.current_callback)
    rospy.Subscriber('/' + LEADER_NAME + '/scan', LaserScan, self.leader_callback)
    self._angles = [0., np.pi / 4., -np.pi / 4., np.pi / 2., -np.pi / 2.]
    self._width = np.pi / 180. * 10.  # 10 degrees cone of view.
    self._current_measurements = [float('inf')] * len(self._angles)
    self._leader_measurements = [float('inf')] * len(self._angles)
    self._indices = None

  def current_callback(self, msg):
    self.callback(msg, self._current_measurements)

  def leader_callback(self, msg):
    self.callback(msg, self._leader_measurements)

  def callback(self, msg, measurement_arr):
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
      measurement_arr[i] = np.percentile(ranges[idx], 10)

  @property
  def ready(self):
    return not np.isnan(self._current_measurements[0]) and not np.isnan(self._leader_measurements[0])

  @property
  def measurements(self):
    return self._current_measurements

  @property
  def leader_measurements(self):
    return self._leader_measurements


class GroundtruthPose(object):
  def __init__(self, name='turtlebot3_burger'):
    rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
    self._pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
    self._leader_pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
    self._name = name

  def callback(self, msg):
    # Store belief of current robot and leader, as it is the only state required
    ind_name = [(i, n) for i, n in enumerate(msg.name) if n == self._name or n == LEADER_NAME]
    if not ind_name:
      raise ValueError('Specified name "{}" does not exist.'.format(self._name + " or " + LEADER_NAME))
    for ind, name in ind_name:
      # Pose for current robot
      pose = np.array(self._pose)
      pose[0] = msg.pose[ind].position.x
      pose[1] = msg.pose[ind].position.y
      _, _, yaw = euler_from_quaternion([
          msg.pose[ind].orientation.x,
          msg.pose[ind].orientation.y,
          msg.pose[ind].orientation.z,
          msg.pose[ind].orientation.w])
      pose[2] = yaw

      if name == self._name:
        self._pose = np.array(pose)
      if name == LEADER_NAME:
        self._leader_pose = np.array(pose)
            
  @property
  def ready(self):
    return not np.isnan(self._pose[0]) and not np.isnan(self._leader_pose[0])

  @property
  def pose(self):
    return self._pose

  @property
  def leader_pose(self):
    return self._leader_pose

def save_error(robot_position, desired_position):
  if ROBOT_ID != LEADER_ID:
  	ERRORS.append(np.linalg.norm(robot_position - desired_position))

def write_error():
  if ROBOT_ID != LEADER_ID:
    labeled_err = {ROBOT_NAME : ERRORS}
    with open('errors.txt', 'a+') as f:
  	  f.write(str(labeled_err))

def run():
  rospy.init_node('obstacle_avoidance')

  # Update control every 50 ms.
  rate_limiter = rospy.Rate(50)

  publisher = rospy.Publisher('/' + ROBOT_NAME + '/cmd_vel', Twist, queue_size=5)
  laser = SimpleLaser(ROBOT_NAME)
  groundtruth = GroundtruthPose(ROBOT_NAME)
  vel_msg = None

  # RRT path
  # If RUN_RRT is False, load the predefined path
  if not RUN_RRT:
    current_path = MAP_PARAMS["RRT_PATH"]
  else:
    current_path = None
  
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
    if not laser.ready or not groundtruth.ready:
      rate_limiter.sleep()
      continue

    LEADER_POSE = groundtruth.leader_pose

    # Compute RRT on the leader only
    if ROBOT_ID == LEADER_ID:
      while not current_path:
        start_node, final_node = rrt.rrt(LEADER_POSE, GOAL_POSITION, occupancy_grid)

        current_path = rrt_navigation.get_path(final_node)
        # print("CURRENT PATH: ", current_path)

      # get the RRT velocity for the leader robot
      position = np.array([LEADER_POSE[X] + EPSILON*np.cos(LEADER_POSE[YAW]),
                           LEADER_POSE[Y] + EPSILON*np.sin(LEADER_POSE[YAW])], dtype=np.float32)
      LEADER_VELOCITY = rrt_navigation.get_velocity(position, np.array(current_path, dtype=np.float32))
    else:
      # Let the leader velocity be 0, since the leader pose will update and the
      # formation velocity will correctly move the robot
      LEADER_VELOCITY = np.array([0., 0.])

    # get the velocity for this robot
    u, w, desired_position = gcv.get_combined_velocity(groundtruth.pose, LEADER_POSE, LEADER_VELOCITY, laser, ROBOT_ID)

    save_error(groundtruth.pose[:2], desired_position)

    vel_msg = Twist()
    vel_msg.linear.x = u
    vel_msg.angular.z = w

    publisher.publish(vel_msg)

    rate_limiter.sleep()


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Runs decentralized formation control')
  parser.add_argument('--id', action='store', default='0', help='Method.', choices=[str(i) for i,_ in enumerate(ROBOT_NAMES)])
  args, unknown = parser.parse_known_args()

  ROBOT_ID = int(args.id)
  ROBOT_NAME = ROBOT_NAMES[ROBOT_ID]

  try:
    run()
  except rospy.ROSInterruptException:
    pass

  write_error()
