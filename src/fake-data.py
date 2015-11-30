#!/usr/bin/env python

import numpy as np

import rospy

from geometry_msgs.msg import Vector3

class FakeData():
  def __init__(self, x_length = 1.4, y_length=0.7,
                     observation_noise = np.array([[  0.00001,  0.0,  0.0,  0.0],
                                                   [  0.0,  0.00001,  0.0,  0.0],
                                                   [  0.0,  0.0,  0.0,  0.0], 
                                                   [  0.0,  0.0,  0.0,  0.0]])):
    self.publisher = rospy.Publisher('/ball_position_topic', Vector3, queue_size = 10)
    rospy.init_node('fakedata')
    rate = rospy.Rate(10)

    self.x_length = x_length
    self.y_length = y_length
    self.observation_noise = observation_noise

    self.last_recorded_time = rospy.get_time()
    self.position = np.array([0.2, 0.23])
    rospy.loginfo(self.position)
    self.velocity = np.array([-0.2,0.01])

    while not rospy.is_shutdown():
#      rospy.loginfo("New data!")
      t = rospy.get_time()
#      rospy.loginfo("New time: {0}".format(t))
      rospy.loginfo(self.position)
      self.advance_time(t - self.last_recorded_time)
      self.last_recorded_time = t
      rospy.loginfo("Finished Position Update")
#     
      sampled_observation_noise = np.random.multivariate_normal(np.zeros(4), self.observation_noise)
      self.publisher.publish(Vector3(self.position[0] + sampled_observation_noise[0], 
        self.position[1] + sampled_observation_noise[1], float('nan')))
      rospy.loginfo("Published Data!")
      rate.sleep()

  def advance_time(self, t):
    t_next_event, wall_number = self.get_next_event()
    print("Next Event in: {0}".format(t_next_event)) 
 
    if t < t_next_event:
      print("No bounce")
      self.position += t * self.velocity
    else:
      t_advanced_so_far = 0.0
      bounce_counter = 1
      while t - t_advanced_so_far > t_next_event:
        print("bounce {0}".format(bounce_counter))
        t_advanced_so_far += t_next_event
 
        self.position += t_next_event * self.velocity 
  
        if wall_number is 0 or wall_number is 1:
          self.velocity[0] *= -1
        else:
          self.velocity[1] *= -1
  
        t_next_event, wall_number = self.get_next_event()
        bounce_counter += 1
 
      self.position += (t - t_advanced_so_far) * self.velocity 


  def get_next_event(self):
    t_left_wall_collision = - self.position[0] / self.velocity[0]
    t_right_wall_collision = (self.x_length - self.position[0]) / self.velocity[0]
    t_bottom_wall_collision = - self.position[1] / self.velocity[1]
    t_top_wall_collision = (self.y_length - self.position[1]) / self.velocity[1]
    print("t_left: {0}".format(t_left_wall_collision))
    print("t_right: {0}".format(t_right_wall_collision))
    print("t_bottom: {0}".format(t_bottom_wall_collision))
    print("t_top: {0}".format(t_top_wall_collision))

    return minimum_positive([t_left_wall_collision, t_right_wall_collision, t_bottom_wall_collision, t_top_wall_collision], 1e-14)


def minimum_positive(a_list, eps):
  current_minimum = float('inf')
  current_index = None
  
  for i, list_element in enumerate(a_list):
    if (list_element > eps) and (list_element < current_minimum):
      current_minimum = list_element
      current_index = i

  return current_minimum, current_index

if __name__ == '__main__':
  try:
    FakeData()
  except rospy.ROSInterruptException :
    pass
  
