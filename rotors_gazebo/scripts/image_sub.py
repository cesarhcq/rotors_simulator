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

#-- Define Tag\n",
id_to_find = 1
marker_size = 40 #-cm

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

    #-- Get the camera calibration\n",
    calib_path = '/home/alantavares/aruco_landing_ws/src/rotors_simulator/'
    camera_matrix = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter = ',')
    camera_distortion = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter = ',')

    try:
      src_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #-- Convert in gray scale\n",
    gray = cv2.cvtColor(src_image, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

    #(rows,cols,channels) = src_image.shape
    #if cols > 60 and rows > 60 :
      #cv2.circle(src_image, (50,50), 10, 255)
  
    #print('cols = {} rows = {}'.format(cols,rows))

    #-- Find all the aruco markers in the image\n",
    corners, ids, rejected = aruco.detectMarkers(image=gray,
                                                  dictionary=aruco_dict,
                                                  parameters=parameters,
                                                  cameraMatrix=camera_matrix,
                                                  distCoeff=camera_distortion)

    if ids != None and ids[0] == id_to_find:
      #-- ret= [rvec,tvec, ?]
      #-- array of rotation and position of each marker in camera frame
      #-- rvec = [[rvec_1, [rvec2], ...]]  attitude of the marker respect to camera frame
      #-- tvec = [[tvec_1, [tvec2], ...]]  position of the marker in camera frame
      ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

      #-- Unpack the output, get only the first\n",
      rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

      #-- Draw the detected marker and put a reference frame over it\n",
      aruco.drawDetectedMarkers(src_image, corners)
      aruco.drawAxis(src_image, camera_matrix, camera_distortion, rvec, tvec, 40)
      
      cv2.imshow("Image-Aruco", src_image)
      #cv2.imshow("Image-Gray", gray)
      cv2.waitKey(3)

    else:
      print('Nothing detected')
      #-- Display the resulting frame\n",
      cv2.imshow("Image-Aruco",src_image)

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