import cv2
from PIL import Image
from util import get_limits

orange = [0, 127, 255]  # orange in BGR colorspace
red = [0, 0, 255]  # red in BGR colorspace (masih belum pas BGR nya)
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()

    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lowerLimit, upperLimit = get_limits(color=orange)

    mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)

    mask_ = Image.fromarray(mask)

    bbox = mask_.getbbox()

    if bbox is not None:
        x1, y1, x2, y2 = bbox
        # x1 and y1 = top left corner
        # x2 and y2 = bottom right corner
        # # Calculate the center coordinates of the rectangle
        # x_center = (x1 + x2) // 2
        # y_center = (y1 + y2) // 2

        frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
        
        # Add text to show the coordinates of the detected object
        text = f"({x1}, {y1}), ({x2}, {y2})"
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 0.5
        fontColor = (255, 255, 255)  # White color in BGR
        lineType = 2
        cv2.putText(frame, text, (x1, y1 - 10), font, fontScale, fontColor, lineType)
        
        # # Draw a red circle at the center of the rectangle
        # center_color = (0, 0, 255)  # Red color in BGR
        # radius = 5
        # cv2.circle(frame, (x_center, y_center), radius, center_color, -1)


    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()

cv2.destroyAllWindows()

