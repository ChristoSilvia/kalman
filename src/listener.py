#!/usr/bin/env python
import numpy as np

import rospy
from geometry_msgs.msg import Vector3

from filter.srv import GetPositionService, GetPositionServiceResponse

def KalmanFilter():
  def __init__(self, position_topic='/ball-position-topic',
                     get_position_service='/predicted-ball-position',
                     x_length=0.5,
                     y_length=1.5,
                     initial_mean = np.array([ 0.25, 0.75, 0.0, 0.0]),
                     initial_covariance = np.array([[1.0,  0.0,  0.0,  0.0],
                                                    [0.0,  1.0,  0.0,  0.0],
                                                    [0.0,  0.0,  1.0,  0.1],
                                                    [0.0,  0.0, 0.04, 0.05]]),
                     process_noise = 0.005*np.eye(4),
                     observation_noise = np.array([[ 0.01,  0.0,  0.0,  0.0],
                                                   [  0.0, 0.01,  0.0,  0.0],
                                                   [  0.0,  0.0,  5.0,  0.0], 
                                                   [  0.0,  0.0,  0.0,  5.0]]), 
                     observation_model = np.array([[ 1.0,  0.0,  0.0,  0.0],
                                                   [ 0.0,  1.0,  0.0,  0.0],
                                                   [ 0.0,  0.0,  0.0,  0.0],
                                                   [ 0.0,  0.0,  0.0,  0.0]])):
    # intialize ros node
    rospy.init_node('kalman-filter')
    rospy.loginfo("Initialized node: 'kalman-filter'")

    # subscribe to source of position info
    self.position_topic = rospy.Subscriber(position_topic, Vector3, self.position_data_update)
    self.prediction_service = rospy.Service(get_position_service, GetPositionService, self.get_predicted_position) 

    # set parameters
    self.x_length = x_length
    self.y_length = y_length
    self.process_noise = process_noise
    self.observation_model = observation_model
    self.observation_noise = observation_noise

    # set variables
    self.time_of_last_data = rospy.get_time()
    self.mean = initial_mean
    self.covariance = initial_covariance

    rospy.spin()


  def position_data_update(self, position):
    # get time advancement matrix
    t_now = rospy.get_time()
    t = t_now - self.time_of_last_data
    self.time_of_last_data = t_now 
   
    time_advancement_matrix = self.update_matrix(t)

    expected_mean = np.dot(time_advancement_matrix, self.mean)
    expected_covariance = np.dot(time_advancement_matrix,
                                 np.dot(self.covariance,
                                        time_advancement_matrix.T)) + self.process_noise

    x,y = position.x, position.y
    difference_from_measurement = np.array([x,y,0,0]) - np.dot(self.observation_model, self.mean)
    difference_covariance = np.dot(self.observation_model, 
                              np.dot(expected_covariance,
                                self.observation_model.T)) + self.observation_noise

    kalman_gain = np.dot(self.expected_covariance,
                    np.dot(self.observation_model.T), 
                      np.linalg.inv(difference_covariance))

    self.mean = expected_mean + np.dot(kalman_gain, difference_from_measurement)
    self.covariance = np.dot(
                        np.eye(4) - np.dot(kalman_gain, self.observation_model),
                        expected_covariance)


  def update_matrix(self, t):
    mean_position = t * self.mean[2:3] + self.mean[0:1]

    transition_matrix = np.array([[ 1.0, 0.0, t, 0.0], 
                                  [ 0.0, 1.0, 0.0, t],
                                  [ 0.0, 0.0, 1.0, 0.0],
                                  [ 0.0, 0.0, 0.0, 1.0]])
    
    if (mean_position[0] < 0.0) or (mean_position[0] > self.x_length):
      transition_matrix[2,2] = -1.0
    
    if (mean_position[1] < 0.0) or (mean_position[1] > self.y_length):
      transition_matrix[3,3] = -1.0 

  def get_predicted_position(self, no_args):
    t = rospy.get_time() - self.time_of_last_data
    update_matrix = self.update_matrix(t)
    prediction_mean = np.dot(update_matrix, self.mean)
    prediction_covariance = np.dot(update_matrix, np.dot(self.covariance, update_matrix)) + self.process_noise
    
    sampled_state_prediction = np.random.multivariate_normal(prediction_mean, prediction_covariance)
    return GetPositionServiceResponse(Vector3(sampled_state_prediction[0],
                                              sampled_state_prediction[1],
                                              float('nan')))

if __name__ == '__main__':
  try:
    KalmanFilter()
  except rospy.ROSInterruptException:
    pass

