#!/usr/bin/env python

import numpy as np

import rospy

from geometry_msgs.msg import Vector3

class FakeData():
  def __init__(self, x_length = 0.5, y_length=1.5,
                     observation_noise = np.array([[ 0.01,  0.0,  0.0,  0.0],
                                                   [  0.0, 0.01,  0.0,  0.0],
                                                   [  0.0,  0.0,  5.0,  0.0], 
                                                   [  0.0,  0.0,  0.0,  5.0]])):
    self.publisher = rospy.Publisher('/ball_position_topic', Vector3, queue_size = 10)
    rospy.init_node('fakedate')
    rate = rospy.Rate(30)

    self.x_length = x_length
    self.y_length = y_length
    self.observation_noise = observation_noise

    self.last_recorded_time = rospy.get_time()
    self.position = np.array([0.3, 0.3])
    self.velocity = np.array([0.01, 0.0042])

    while not rospy.is_shutdown():
      rospy.loginfo("New data!")
      t = rospy.get_time()
      rospy.loginfo("New time: {0}".format(t))
      self.advance_time(t - self.last_recorded_time)
      self.last_recorded_time = t
     
      sampled_observation_noise = np.random.multivariate_normal(np.zeros(4), self.observation_noise)
      self.publisher.publish(Vector3(self.position[0], self.position[1], float('nan')))
      rospy.loginfo("Published Data!")
      rate.sleep()

  def advance_time(self, t):
    t_left = t
    while t_left > 0:
      time, which_wall = self.get_next_event()
      
      if time >= t_left:
        self.position += self.velocity * t_left
        t_left = 0.0
      else:
        self.position += self.velocity * time
        if (which_wall is 0) or (which_wall is 1):
          self.velocity[0] *= -1
        else:
          self.velocity[1] *= -1
        t_left -= time  

  def get_next_event(self):
    t_left_wall_collision = - self.position[0] / self.velocity[0]
    t_right_wall_collision = self.x_length - self.position[0] / self.velocity[0]
    t_bottom_wall_collision = - self.position[1] / self.velocity[1]
    t_top_wall_collision = self.y_length - self.position[1] / self.velocity[1]

    min_time, which_wall = minimum_positive([t_left_wall_collision,
                                             t_right_wall_collision,
                                             t_bottom_wall_collision,
                                             t_top_wall_collision ])
    return min_time, which_wall


def minimum_positive(a_list):
  current_minimum = a_list[0]
  current_index = 0
  
  for i in xrange(1,len(a_list)):
    if (a_list[i] > 0) and (a_list[i] < current_minimum):
      current_minimum = a_list[i]
      current_index = i

  return current_minimum, current_index

if __name__ == '__main__':
  try:
    FakeData()
  except rospy.ROSInterruptException :
    pass
  
