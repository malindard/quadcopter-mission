import cv2

import numpy as np
 
def nothing(x):
    pass
 
cap = cv2.VideoCapture(0)
 
cv2.namedWindow("live transmission", cv2.WINDOW_AUTOSIZE)
 
cv2.namedWindow("Tracking")
cv2.createTrackbar("LH", "Tracking", 0, 255, nothing)
cv2.createTrackbar("LS", "Tracking", 0, 255, nothing)
cv2.createTrackbar("LV", "Tracking", 0, 255, nothing)
cv2.createTrackbar("UH", "Tracking", 255, 255, nothing)
cv2.createTrackbar("US", "Tracking", 255, 255, nothing)
cv2.createTrackbar("UV", "Tracking", 255, 255, nothing)
# l_h, l_s, l_v = 92, 57, 50
# u_h, u_s, u_v = 142, 153, 178

l_h, l_s, l_v = 5, 50, 50
u_h, u_s, u_v = 15, 255, 255

# l_h, l_s, l_v = 10, 100, 20 (kalo deteksi orange lewat foto lebih akurat, kalo kamera ga terlalu akurat)
# u_h, u_s, u_v = 25, 255, 255
 
while True:
    
    ret,frame = cap.read()
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
    l_h = cv2.getTrackbarPos("LH", "Tracking")
    l_s = cv2.getTrackbarPos("LS", "Tracking")
    l_v = cv2.getTrackbarPos("LV", "Tracking")
     
    u_h = cv2.getTrackbarPos("UH", "Tracking")
    u_s = cv2.getTrackbarPos("US", "Tracking")
    u_v = cv2.getTrackbarPos("UV", "Tracking")
 
    l_b = np.array([l_h, l_s, l_v])
    u_b = np.array([u_h, u_s, u_v])

    # l_b = np.array([118, 78, 42])
    # u_b = np.array([222, 114, 156])
    
    mask = cv2.inRange(hsv, l_b, u_b)
    cnts, _ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
 
    for c in cnts:
        area=cv2.contourArea(c)
        if area>2000:
            cv2.drawContours(frame,[c],-1,(0,0,255),3)
            M=cv2.moments(c)
            cx=int(M["m10"]/M["m00"])
            cy=int(M["m01"]/M["m00"])
 
            cv2.circle(frame,(cx,cy),7,(255,255,255),-1)
            print(cx,cy)
            cv2.putText(frame,"orange",(cx-20, cy-20),cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2) 
    res = cv2.bitwise_and(frame, frame, mask=mask)
 
    cv2.imshow("live transmission", frame)
    cv2.imshow("mask", mask)
    cv2.imshow("res", res)
    key=cv2.waitKey(5)
    if key==ord('q'):
        break
    
cv2.destroyAllWindows()