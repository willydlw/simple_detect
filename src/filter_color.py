#!/usr/bin/env python

import cv2 
import numpy as np

# HSV filter
redLow = np.array([0, 160, 5])
redHigh = np.array([180, 255, 255])


# path  
path = 'angled_backpack1.png'

try:
   # Reading an image in default mode 
   src = cv2.imread(path)
except cv2.error as e:
   print('imread failure') 

   
# Window name in which image is displayed 
window_name = 'Image'

hsv_frame = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
red_mask = cv2.inRange(hsv_frame, redLow, redHigh)
red_frame = cv2.bitwise_and(src, src, mask=red_mask)


# Displaying the image 
cv2.imshow("Source", src)
cv2.imshow("hsv", hsv_frame)
cv2.imshow("red mask", red_mask)
cv2.imshow("Red", red_frame)
  
 
cv2.waitKey(0)
cv2.destroyAllWindows()
