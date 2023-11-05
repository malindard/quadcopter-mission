import cv2
import numpy as np

img = cv2.imread('drop.jpg')

# ORANGE_MIN = np.array([5, 50, 50],np.uint8) # orange
# ORANGE_MAX = np.array([15, 255, 255],np.uint8)

# ORANGE_MIN = np.array([10, 100, 20],np.uint8) # orange
# ORANGE_MAX = np.array([25, 255, 255],np.uint8)

RED_MIN = np.array([0, 100, 100],np.uint8) # red
RED_MAX = np.array([179, 255, 255],np.uint8)

hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

frame_threshed = cv2.inRange(hsv_img, RED_MIN, RED_MAX)
cv2.imwrite('drop3.jpg', frame_threshed)