import cv2
from PIL import Image
from util import get_limits

orange = [0, 127, 255]  # orange in BGR colorspace
red = [0, 0, 255]  # red in BGR colorspace (masih belum pas BGR nya)
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()

    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    orangeLower, orangeUpper = get_limits(color=orange)
    redLower, redUpper = get_limits(color=red)

    orangeMask = cv2.inRange(hsvImage, orangeLower, orangeUpper)
    redMask = cv2.inRange(hsvImage, redLower, redUpper)

    # Combine both masks to get a single mask for both red and orange colors
    # Find contours in the orange mask and draw rectangles around each detected object
    orange_contours, _ = cv2.findContours(orangeMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in orange_contours:
        x1, y1, w, h = cv2.boundingRect(contour)
        x2, y2 = x1 + w, y1 + h
        frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 165, 255), 5)  # Orange color

    # Find contours in the red mask and draw rectangles around each detected object
    red_contours, _ = cv2.findContours(redMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in red_contours:
        x1, y1, w, h = cv2.boundingRect(contour)
        x2, y2 = x1 + w, y1 + h
        frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 5)  # Red color

        # # Draw a red circle at the center of the rectangle
        # center_color = (0, 0, 255)  # Red color in BGR
        # radius = 5
        # cv2.circle(frame, (x_center, y_center), radius, center_color, -1)


    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()

cv2.destroyAllWindows()
