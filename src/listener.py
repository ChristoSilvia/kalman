#!/usr/bin/env python
import numpy as np

import rospy
from geometry_msgs.msg import Vector3

from kalman_filter.srv import GetPositionService, GetPositionServiceResponse
from kalman_filter.msg import moments

class KalmanFilter():
  def __init__(self, position_topic='/ball_position_topic',
                     get_position_service='/predicted_ball_position',
                     moments_topic_name='/ball_state_moments',
                     x_length=1.4,
                     y_length=0.7,
                     initial_mean = np.array([ 0.25, 0.75, 0.0, 0.0]),
                     initial_covariance = np.array([[0.01,  0.0,  0.0,  0.0],
                                                    [0.0,  0.01,  0.0,  0.0],
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
    rospy.init_node('kalman_filter')
    rospy.loginfo("Initialized node: 'kalman-filter'")


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

    # subscribe to source of position info
    self.position_topic = rospy.Subscriber(position_topic, Vector3, self.position_data_update)
    self.prediction_service = rospy.Service(get_position_service, GetPositionService, self.get_predicted_position) 
    self.moments_publisher = rospy.Publisher(moments_topic_name, moments, queue_size=10)

    rospy.spin()


  def position_data_update(self, position):
    # get time advancement matrix
    t_now = rospy.get_time()
    t = t_now - self.time_of_last_data
    self.time_of_last_data = t_now 
    print("Time Since Last Measurement")
    print(t)
   
    time_advancement_matrix = self.update_matrix(t)
    print("Time Advancement Matrix: ")
    print(time_advancement_matrix)

    expected_mean = np.dot(time_advancement_matrix, self.mean)
    print("Expected Mean:")
    print(expected_mean)

    expected_covariance = np.dot(time_advancement_matrix,
                                 np.dot(self.covariance,
                                        time_advancement_matrix.T)) + self.process_noise
    print("Expected Covariance: ")
    print(expected_covariance)

    x,y = position.x, position.y
    difference_from_measurement = np.array([x,y,0,0]) - np.dot(self.observation_model, self.mean)

    print("Difference from Measurement: ")
    print(difference_from_measurement)

    difference_covariance = np.dot(self.observation_model, 
                              np.dot(expected_covariance,
                                self.observation_model.T)) + self.observation_noise
    print("Difference Covariance: ")
    print(difference_covariance)

    kalman_gain = np.dot(expected_covariance,
                    np.dot(self.observation_model.T, 
                      np.linalg.inv(difference_covariance)))
    print("Kalman Gain")
    print(kalman_gain)

    self.mean = expected_mean + np.dot(kalman_gain, difference_from_measurement)
    print("New Mean: ")
    print(self.mean)

    self.covariance = np.dot(
                        np.eye(4) - np.dot(kalman_gain, self.observation_model),
                        expected_covariance)
    print("New Covariance")
    print(self.covariance)

    self.moments_publisher.publish(moments(list(self.mean), list(self.covariance.flatten())))


  def update_matrix(self, t):
    mean_position = t * self.mean[2:] + self.mean[:2]

    transition_matrix = np.array([[ 1.0, 0.0, t, 0.0], 
                                  [ 0.0, 1.0, 0.0, t],
                                  [ 0.0, 0.0, 1.0, 0.0],
                                  [ 0.0, 0.0, 0.0, 1.0]])
    
#     if (mean_position[0] < 0.0) or (mean_position[0] > self.x_length):
#       transition_matrix[2,2] = -1.0
#     
#     if (mean_position[1] < 0.0) or (mean_position[1] > self.y_length):
#       transition_matrix[3,3] = -1.0 

    return transition_matrix

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
  np.set_printoptions(precision=4)
  try:
    KalmanFilter()
  except rospy.ROSInterruptException:
    pass

