import cv2
import numpy as np

frame1 = cv2.imread('flow_x.jpg')
frame2 = cv2.imread('flow_xy.jpg')
frame3 = cv2.imread('flow_y_00171.jpg')

print frame1.shape
print frame2.shape
print frame3.shape
