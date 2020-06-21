#! /usr/bin/env python

import roslib
import rospy
import cv2
import numpy as np 
import os 

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class BlobDetect(object):

   def __init__(self):

      self.bridge_object = CvBridge() 
      self.image_sub = rospy.Subscriber("/X1/front/image_raw", Image, self.camera_callback)

      self.redMin = np.array([0, 160, 5])
      self.redMax = np.array([180, 255, 255])
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
      params.minThreshold = 0
      params.maxThreshold = 255

      # filter by area
      params.filterByArea = True
      params.minArea = 100           # pixels


      # filter by circularity
      # circularity is defined as ( 4 pi area) / perimeter^2
      # Circle is 1. Square is 0.785.
      # Jansport Big Student backpack, color redtape 
      # 13 x 9.8 x 16.9 inches
      params.filterByCircularity = True 
      params.minCircularity = 0.7 


      # filter by inertia ratio: lines have inertial 0, circles 1
      params.filterByInertia = True 
      params.minInertiaRatio = 0.3
      params.maxInertiaRatio = 1.0


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