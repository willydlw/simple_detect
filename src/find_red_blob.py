#! /usr/bin/env python

import roslib
import rospy
import cv2
import numpy as np 

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class BlobDetect(object):

   def __init__(self):

      self.bridge_object = CvBridge() 
      self.image_sub = rospy.Subscriber("/X1/front/image_raw", Image, self.camera_callback)

      self.redMin = np.array([0, 160, 5])
      self.redMax = np.array([180, 255, 255])

   def camera_callback(self, data):
      try:
         cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
      except CvBridgeError as e:
         print(e)

      hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
      red_mask = cv2.inRange(hsv_frame, self.redMin, self.redMax)
      red_frame = cv2.bitwise_and(cv_image, cv_image, mask=red_mask)

      cv2.imshow("Image window", cv_image)
      cv2.imshow("red mask", red_mask)
      cv2.imshow("Red", red_frame)
      cv2.waitKey(1)
      
def main():
   blob = BlobDetect()
   rospy.init_node("blob_detect_node", anonymous=True)
   try:
      rospy.spin()
   except KeyboardInterrupt:
      print("Shutting down")

   cv2.destroyAllWindows()

if __name__ == '__main__':
   main()