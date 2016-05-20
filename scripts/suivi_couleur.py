#!/usr/bin/env python
from __future__ import print_function

#import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from distutils.version import LooseVersion


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_out",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_topic_in",Image,self.callback)

  # Callback when a new image arrived
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

  # Our operations on the frame come here
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    lower_bound = np.array([330/2,90,30], dtype=np.uint8)
    upper_bound = np.array([360/2,255,255], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_bound, upper_bound);
    # on epure l'image : 
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # calcule le centre de masse de l'image binaire obtenue (donc de l'objet d'interet)
    M = cv2.moments(mask)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    # trace un cercle au centre de masse de l'objet
    center = (cx,cy)
    thickness = 5
    radius = 10
    cv2.circle(mask, center, radius, 150, thickness)
    cv2.circle(cv_image, center, radius, 255, thickness);


    # Display
    cv2.imshow('sift detection',mask)
    cv2.waitKey(3)

    # display
    cv2.imshow("Image with circle", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args): 
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  print("OpenCV Version: {}".format(cv2.__version__))
  print("Python Version: {}".format(sys.version))
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
