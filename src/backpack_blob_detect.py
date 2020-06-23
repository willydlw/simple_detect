#! /usr/bin/env python

import roslib
import rospy
import cv2
import numpy as np 
import os 

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class BlobDetect(object):

   # Import ROS parameters from the "params.yaml" file.
   # Access these variables in class functions with self:
   # i.e. self.CONSTANT
   SCAN_TOPIC = rospy.get_param('blob_detect/scan_topic')
   HUE_MIN = rospy.get_param('blob_detect/hue_min')
   SAT_MIN = rospy.get_param('blob_detect/sat_min')
   VAL_MIN = rospy.get_param('blob_detect/val_min')

   HUE_MAX = rospy.get_param('blob_detect/hue_max')
   SAT_MAX = rospy.get_param('blob_detect/sat_max')
   VAL_MAX = rospy.get_param('blob_detect/val_max')

   # Blob detect parameters
   MIN_THRESHOLD = rospy.get_param('blob_detect/min_threshold')
   MAX_THRESHOLD = rospy.get_param('blob_detect/max_threshold')

   FILTER_BY_AREA = rospy.get_param('blob_detect/filterByArea')
   MIN_AREA = rospy.get_param('blob_detect/minArea')

   FILTER_BY_CIRCULARITY = rospy.get_param('blob_detect/filterByCircularity')
   MIN_CIRCULARITY = rospy.get_param('blob_detect/minCircularity')

   FILTER_BY_INERTIA = rospy.get_param('blob_detect/filterByInertia')
   MIN_INERTIA_RATIO = rospy.get_param('blob_detect/minInertiaRatio')
   MAX_INERTIA_RATIO = rospy.get_param('blob_detect/maxInertiaRatio')

   def __init__(self):

      self.bridge_object = CvBridge() 
      self.image_sub = rospy.Subscriber(self.SCAN_TOPIC, Image, self.camera_callback)

      self.redMin = np.array([self.HUE_MIN, self.SAT_MIN, self.VAL_MIN])
      self.redMax = np.array([self.HUE_MAX, self.SAT_MAX, self.VAL_MAX])
      self.writeImageDirectory = 'home/catkin_ws/src/simple_detect/images/blobs_found/'
      self.writeImageFileName = 'blob'
      self.imageCount = 0

   def camera_callback(self, data):
      try:
         cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
      except CvBridgeError as e:
         print(e)

      hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
      red_mask = cv2.inRange(hsv_frame, self.redMin, self.redMax)
      red_frame = cv2.bitwise_and(cv_image, cv_image, mask=red_mask)

      # make inRange areas larger (fills in missing white pixels)
      dilated_mask = cv2.dilate(red_mask, None, iterations=1)

      # set up blob detector parameters
      # TODO: do we have to set up these parameters each time?
      # would be more efficient to set them up once and then use in the callback
      params = cv2.SimpleBlobDetector_Params()

      # thresholds are used in grayscale images. The mask is a binary black and white
      # image which falls into the grayscale range 0-255
      # Values including and below the minThreshold are set to zero. 
      # values above the maxThreshold are set to zero
      params.minThreshold = self.MIN_THRESHOLD
      params.maxThreshold = self.MAX_THRESHOLD

      # filter by area
      params.filterByArea = self.FILTER_BY_AREA
      params.minArea = self.MIN_AREA           # pixels


      # filter by circularity
      # circularity is defined as ( 4 pi area) / perimeter^2
      # Circle is 1. Square is 0.785.
      # Jansport Big Student backpack, color redtape 
      # 13 x 9.8 x 16.9 inches
      params.filterByCircularity = self.FILTER_BY_CIRCULARITY
      params.minCircularity = self.MIN_CIRCULARITY 


      # filter by inertia ratio: lines have inertial 0, circles 1
      params.filterByInertia = self.FILTER_BY_INERTIA
      params.minInertiaRatio = self.MIN_INERTIA_RATIO
      params.maxInertiaRatio = self.MAX_INERTIA_RATIO


      detector = cv2.SimpleBlobDetector_create(params)
      
      # Detect blobs.
      reversemask=255 - dilated_mask
      keypoints = detector.detect(reversemask)
      if keypoints:
         print "found %d blobs" % len(keypoints)
         if len(keypoints) > 4:
            # if more than four blobs, keep the four largest
            keypoints.sort(key=(lambda s: s.size))
            keypoints=keypoints[0:3]
      else:
         print "no blobs"

      # Draw green circles around detected blobs
      im_with_keypoints = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

      # Draw text on image
      text = "Number Blobs: " + str(len(keypoints)) 
      cv2.putText(im_with_keypoints, text, (20, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 100, 255), 2) 

      # draw white circle in center of blob
      for k in keypoints:
         x = k.pt[0]
         y = k.pt[1]
         print('blob center, x: %u, y: %u' % (x,y))
         # draw a circle to show blob centerpoint x,y
         # cv2.circle(image, center coordinates, radius, color, thickness)
         im_with_keypoints = cv2.circle(im_with_keypoints, (int(x),int(y)), 5, (255,255,255), 2)

      # write images to files
      if keypoints:
         filename = self.writeImageFileName + str(self.imageCount) + ".jpg"
         cv2.imwrite(os.path.join(self.writeImageDirectory, filename), im_with_keypoints)
         cv2.imwrite(filename, im_with_keypoints)
         self.imageCount += 1
         print('imageCount: %d' % self.imageCount)

      # display images
      cv2.imshow("original", cv_image)
      cv2.imshow("dilated mask", dilated_mask)
      cv2.imshow("red", red_frame)
      cv2.imshow("keypoints", im_with_keypoints)   

      # imshow should be followed by waitKey or image will not be displayed
      # displays image for specified milliseconds 
      cv2.waitKey(1)

def myshutdown():
   print 'ros shutting down'
      
def main():
   blob = BlobDetect()
   rospy.init_node("blob_detect_node", anonymous=True)
   rospy.on_shutdown(myshutdown)
   while not rospy.is_shutdown():
      try:
         rospy.spin()
      except KeyboardInterrupt:
         print("Shutting down")

   cv2.destroyAllWindows()

if __name__ == '__main__':
   main()