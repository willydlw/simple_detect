import math


# circularity is defined as ( 4 pi area) / perimeter^2
# Circle is 1. Square is 0.785.
# Jansport Big Student backpack, color redtape 
# 13 x 9.8 x 16.9 inches

width = 13
depth = 9.8 
height = 16.9


front_view_perimeter = 2 * width + 2 * height
front_view_area = width * height 

side_view_perimeter = 2 * depth + 2 * height
side_view_area = depth * height 

top_view_perimeter = 2 * width + 2 * depth
top_view_area = depth * width 


front_view_circularity = (4 * math.pi * front_view_area) / (front_view_perimeter * front_view_perimeter)
side_view_circularity = (4 * math.pi * side_view_area) / (side_view_perimeter * side_view_perimeter)
top_view_circularity = (4 * math.pi * top_view_area) / (top_view_perimeter * top_view_perimeter)


print("front_view_circularity: %.3f" % front_view_circularity)
print("side_view_circularity:  %.3f" % side_view_circularity)
print("top_view_circularity:   %.3f" % top_view_circularity)