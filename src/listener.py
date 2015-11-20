#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

from baxter_core_msgs.srv import ListCameras

def callback(image):
  screen.publish(image)
  rospy.loginfo("Recieved image")

def listener():
  global screen

  screen = rospy.Publisher('/robot/xdisplay', Image, queue_size=10)

  rospy.init_node('listener')
  rospy.loginfo('initialized node')

  rospy.wait_for_service('/cameras/list')
  rospy.loginfo("List Camera Service detected")

  rospy.ServiceProxy('/cameras/list', ListCameras)
  rospy.loginfo("Initialized Service Proxy")

  rospy.Subscriber('/cameras/left_hand_camera/image', Image, callback)

  rospy.spin()

if __name__ == '__main__':
  listener()

