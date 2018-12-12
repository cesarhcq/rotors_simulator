#!/usr/bin/env python
from __future__ import print_function
 
import roslib
roslib.load_manifest('rotors_gazebo')
import sys, time, math
import rospy
import cv2

# numpy and scipy
import numpy as np
import cv2.aruco as aruco

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist


class landing_aruco:
 
  def __init__(self):
   
    #-- Create a topic "aruco_results"
    self.pose_pub = rospy.Publisher("bebop2/camera_base/aruco_results",Twist, queue_size=10)    
  
    self.image_sub = rospy.Subscriber("bebop2/camera_base/image_raw",Image,self.callback)


def main(args):

  ic = landing_aruco()
  #-- Name of node
  rospy.init_node('landing_aruco', anonymous=True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

###############################################################################

  if __name__ == '__main__':
       main(sys.argv)