#! /usr/bin/env python

import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class ColorFilter:
   SCAN_TOPIC = rospy.get_param('/color_filter/scantopic')
   COLORSPACE = rospy.get_param('/color_filter/colorspace')
   

   def __init__(self):
      
      self.bridge_object = CvBridge() 
      self.image_sub = rospy.Subscriber(self.SCAN_TOPIC, Image, self.camera_callback)

      self.trackbarWindowName = 'inRange Filter'

      self.setup_trackbars()

   def camera_callback(self, data):
      try:
         cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
      except CvBridgeError as e:
         rospy.logerr("CvBridgeError: %s" % e)

      if self.COLORSPACE == 'HSV':
         color_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
      else:
         color_image = cv_image.copy()
      
      # trackbar value for inRange filter
      v1min, v2min, v3min, v1max, v2max, v3max = self.get_trackbar_values()
      colorMin = np.array([v1min, v2min, v3min])
      colorMax = np.array([v1max, v2max, v3max])

      inRange_mask = cv2.inRange(color_image, colorMin, colorMax)
      inRange_frame = cv2.bitwise_and(cv_image, cv_image, mask=inRange_mask)

      # display images
      cv2.imshow("original", cv_image)
      cv2.imshow("inRange mask", inRange_mask)
      cv2.imshow(self.trackbarWindowName, inRange_frame)
      
      # imshow should be followed by waitKey or image will not be displayed
      # displays image for specified milliseconds 
      cv2.waitKey(1)


   def slider_callback(self, value):
      pass


   def setup_trackbars(self):
      cv2.namedWindow(self.trackbarWindowName, 0)
   
      for j in self.COLORSPACE:
         for i in ["MIN", "MAX"]:
            v = 0 if i == 'MIN' else 255
            cv2.createTrackbar("%s_%s" % (j,i), self.trackbarWindowName, v, 255, self.slider_callback)

   def get_trackbar_values(self):
      values = []
      
      for i in ["MIN", "MAX"]:
         for j in self.COLORSPACE:
            v = cv2.getTrackbarPos("%s_%s" % (j,i), self.trackbarWindowName)
            values.append(v)

      return values 

   def shutdown(self):
      
      v1min, v2min, v3min, v1max, v2max, v3max = self.get_trackbar_values()

      file1 = open("/home/diane/Desktop/trackbarValues.txt", "w")
      if self.COLORSPACE == 'HSV':
         file1.write("hue min, sat min, val min\n")
      else:
         file1.write("red min, green min, blue min\n")

      minstr = str(v1min) + ", " + str(v2min) + ", " + str(v3min) + "\n"
      file1.write(minstr)

      if self.COLORSPACE == 'HSV':
         file1.write("hue max, sat max, val max\n")
      else:
         file1.write("red max, green max, blue max\n")

      maxstr = str(v1max) + ", " + str(v2max) + ", " + str(v3max) + "\n"
      file1.write(maxstr)
      file1.close()
   

      
def main():
   cf = ColorFilter()
   rospy.init_node("color_filter_node", anonymous=True)
   
   while not rospy.is_shutdown():
      try:
         rospy.spin()
      except KeyboardInterrupt:
         print("Shutting down")
   cf.shutdown()
   cv2.destroyAllWindows()

if __name__ == '__main__':
   main()