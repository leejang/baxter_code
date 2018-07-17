import cv2

cur_image = 'hands-right.png'
target_image = 'rhand.jpg'
img = cv2.imread(cur_image)
small = cv2.resize(img, (0,0), fx=0.2, fy=0.2)
cv2.imwrite(target_image, small, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
