#!/usr/bin/env python

import cv2 
import numpy as np
import os 


# HSV filter
redLow = np.array([0, 160, 5])
redHigh = np.array([180, 255, 255])


imageCount = 0

# r in front of string makes it a raw string, treats everything as a character
# likely not necessary as / does not signify escape sequence
writeImageDirectory = r'../images/blobs_found/'
writeImageFileName = r'blob'


# path  
path = '../images/artifact_validator/backpack1.png'

try:
   # Reading an image in default mode 
   frame = cv2.imread(path)
except cv2.error as e:
   print('imread failure') 

# convert to hsv color space
hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# filter for range of hue, sat, value
red_mask = cv2.inRange(hsv_frame, redLow, redHigh)

# only used for display
red_frame = cv2.bitwise_and(frame, frame, mask=red_mask)

# make inRange areas larger (fills in missing white pixels)
red_mask = cv2.dilate(red_mask, None, iterations=1)

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
reversemask=255-red_mask
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
im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


# Draw text on image
text = "Number of Circular Blobs: " + str(len(keypoints)) 
cv2.putText(im_with_keypoints, text, (20, 300), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 100, 255), 2) 


if keypoints:
   filename = writeImageFileName + str(imageCount) + ".jpg"
   cv2.imwrite(os.path.join(writeImageDirectory, filename), im_with_keypoints)
   imageCount += 1


# open windows with original image, mask, and image with keypoints marked
cv2.imshow('original',frame)
cv2.imshow('mask',red_mask)
cv2.imshow('inRange filtered',red_frame)     
cv2.imshow("Keypoints for " + path, im_with_keypoints)            
   
k = cv2.waitKey(0)

cv2.destroyAllWindows()
