import cv2, time
import numpy as np
import imutils

class Verified:
    def __init__(self, id=None, x=None, y=None, confirmed=False):
        self.id = id
        self.x = x
        self.y = y
        self.confirmed = confirmed

def objectDetection(mission):
    if mission == 1:
        lowerLimit = np.array([10, 100, 20]) # surprisingly, this is more stable than the previous one lol
        upperLimit = np.array([25, 255, 255])
    elif mission == 2:
        lowerLimit = np.array([170, 50, 50]) # THIS IS THE ONE
        upperLimit = np.array([180, 255, 255])

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    time.sleep(2)
    while True:
        ret, frame = cap.read()  # read current frame
        if not ret:
            break

        blurred = cv2.GaussianBlur(frame, (11, 11), 0) # blur the frame
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV) # convert it to HSV color space

        mask = cv2.inRange(hsv, lowerLimit, upperLimit) # make mask for orange color
        mask = cv2.erode(mask, None, iterations=2) # perform erosions
        mask = cv2.dilate(mask, None, iterations=2) # and dilations on the mask to remove any small blobs
        
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None # initialize the current (x,y) center of the ball

        if len(cnts) > 0: # will run if at least one contour was found
            c = max(cnts, key=cv2.contourArea) # find the largest contour in the mask
            ((x, y), radius) = cv2.minEnclosingCircle(c) # find the minimum enclosing circle
            M = cv2.moments(c) # calculate the center of the contour
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])) # only proceed if the radius meets a minimum size
            if radius > 10:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1) # draw the circle and centroid on the frame
                #print("x: ", round(x), "y: ", round(y))
                cv2.putText(frame, f"x: {round(x)}, y: {round(y)}", (int(x - radius), int(y - radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)



        cv2.imshow("Frame", frame)  # show the frame to screen
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):  # if the 'q' key is pressed, stop the loop
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    a = int(input("Mission: "))
    objectDetection(a)
