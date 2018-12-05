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

#-- Define the Aruco dictionary\n",
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters =  aruco.DetectorParameters_create()

###############################################################################
 
class image_converter:
 
  def __init__(self):
    #self.image_pub = rospy.Publisher("image_topic_2",Image)
  
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("bebop2/camera_base/image_raw",Image,self.callback)

###############################################################################
   
  def callback(self,data):

    try:
      src_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
   
    (rows,cols,channels) = src_image.shape
    if cols > 60 and rows > 60 :
      #cv2.circle(src_image, (50,50), 10, 255)
  
      print('cols = {} rows = {}'.format(cols,rows))
      
      cv2.imshow("Image window", src_image)
      cv2.waitKey(3)
   
    #try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(src_image, "bgr8"))
    #except CvBridgeError as e:
    #  print(e)

###############################################################################

def isRotationMatrix(R):

  #-- transpose the matrix R
  Rt = np.transpose(R)

  #-- verify if Rt could be identity
  shouldBeIdentity = np.dot(Rt, R)

  #-- create a identity
  I = np.identity(3, dtype=R.dtype)
  n = np.linalg.norm(I - shouldBeIdentity)

  return n < 1e-6

###############################################################################

def main(args):

  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

###############################################################################
   
if __name__ == '__main__':
       main(sys.argv)