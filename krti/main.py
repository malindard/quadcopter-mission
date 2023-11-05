############################# drone libraries #############################

from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time, math

############################# color detection libraries #############################

import cv2
import numpy as np
import imutils
import threading

############################# GPIO setting #############################

import RPi.GPIO as GPIO

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

############################# sensor setting ######################################

# Define the TRIG and ECHO pins for each sensor
right_TRIG = 17
right_ECHO = 18
left_TRIG = 27
left_ECHO = 23
forward_TRIG = 22
forward_ECHO = 24
# down_TRIG = 10
# down_ECHO = 25

# Set up the GPIO pins
GPIO.setup(right_TRIG, GPIO.OUT)
GPIO.setup(right_ECHO, GPIO.IN)
GPIO.setup(left_TRIG, GPIO.OUT)
GPIO.setup(left_ECHO, GPIO.IN)
GPIO.setup(forward_TRIG, GPIO.OUT)
GPIO.setup(forward_ECHO, GPIO.IN)
# GPIO.setup(down_TRIG, GPIO.OUT)
# GPIO.setup(down_ECHO, GPIO.IN)

############################# magnet setting #############################

magnet_PIN = 16
GPIO.setup(magnet_PIN, GPIO.OUT)

############################# motor setting #############################

motor_pin1 = 12  # Example GPIO pin for motor control
motor_pin2 = 20  # Example GPIO pin for motor control

GPIO.setup(motor_pin1, GPIO.OUT)
GPIO.setup(motor_pin2, GPIO.OUT)

############################# font color #############################

font = cv2.FONT_HERSHEY_DUPLEX
color_green = 0,255,0
color_red   = 0,0,255
color_white = 255,255,255
color_yellow = 0, 255, 255

############################# global variable #############################

x_max = 640     # width of pi camera image
y_max = 480     # height of pi camera image
x_center = x_max / 2  # center value used for readability/ease of editing
y_center = y_max / 2  # center value used for readability/ease of editing

# Target distance for left and right sensor
targetDistanceSide = 60.00 # in centimeters

# Target distance for front sensor
targetDistanceFront = 60.00 # in centimeters

############################# connect to vehicle #############################

# vehicle = connect('/dev/ttyS0', wait_ready=False, baud=921600) #Konek ke Serial0(Raspberry Pi 4)
# vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
# vehicle = connect('COM9', wait_ready=True, baud=57600) #Konek ke port COMx(WINDOWS) via Telemetry
vehicle = connect('tcp:127.0.0.1:5763', wait_ready=True, baud=57600) #konek via TCP(SITL)

############################# verified class #############################

class Verified:
    def __init__(self, x=None, y=None, confirmed=False):
        self.x = x
        self.y = y
        self.confirmed = confirmed

verified = Verified() #inisiasi Objek Verified sebagai verified

############################# sensor class #############################

class Sensor:
    def __init__(self, right=None, left=None, forward=None):
        self.right = right
        self.left = left
        self.forward = forward

sensor = Sensor() #inisiasi Objek sensor sebagai sensor

############################# FUNCTIONS #############################
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print(" Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print(" Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print(" Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def send_to(latitude, longitude, altitude):
    # Will send the drone to desired location, when the vehicle is in GUIDED mode.
    if vehicle.mode.name == "GUIDED":
        location = LocationGlobalRelative(latitude, longitude, float(altitude))
        vehicle.simple_goto(location)
        time.sleep(10)

def destination_location(homeLattitude, homeLongitude, distance, bearing):
    #Radius of earth in metres
    R = 6371e3

    rlat1 = homeLattitude * (math.pi/180) 
    rlon1 = homeLongitude * (math.pi/180)

    d = distance

    #Converting bearing to radians
    bearing = bearing * (math.pi/180)

    rlat2 = math.asin((math.sin(rlat1) * math.cos(d/R)) + (math.cos(rlat1) * math.sin(d/R) * math.cos(bearing)))
    rlon2 = rlon1 + math.atan2((math.sin(bearing) * math.sin(d/R) * math.cos(rlat1)) , (math.cos(d/R) - (math.sin(rlat1) * math.sin(rlat2))))

    #Converting to degrees
    rlat2 = rlat2 * (180/math.pi) 
    rlon2 = rlon2 * (180/math.pi)

    # Lat and Long as an Array
    location = [rlat2, rlon2]

    return location

def navigation(nav, dis):
    # This function will navigate the vehicle based on the input
    # nav = input(" Where you want to go?\n").upper()
    # dis = float(input(" The distance?\n").upper())

    # Vehicle Location
    angle = int(vehicle.heading)
    loc   = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)

    # Target Distance in meters
    target_distance = dis

    if nav == 'FORWARD':
        front = angle + 0
        new_loc = destination_location(homeLattitude = loc[0], homeLongitude = loc[1], distance = target_distance, bearing = front)
        print(" Ready to go forward as far as %f meter" % target_distance)
        send_to(new_loc[0], new_loc[1], loc[2])

    if nav == 'BACKWARD':
        back = angle + 180
        new_loc = destination_location(homeLattitude = loc[0], homeLongitude = loc[1], distance = target_distance, bearing = back)
        print(" Ready to go backward as far as %f meter" % target_distance)
        send_to(new_loc[0], new_loc[1], loc[2])

    if nav == 'RIGHT':
        right = angle + 90
        new_loc = destination_location(homeLattitude = loc[0], homeLongitude = loc[1], distance = target_distance, bearing = right)
        print(" Ready to turn right for %f meter" % target_distance)
        send_to(new_loc[0], new_loc[1], loc[2])

    if nav == 'LEFT':
        left = angle -90
        new_loc = destination_location(homeLattitude = loc[0], homeLongitude = loc[1], distance = target_distance, bearing = left)
        print(" Ready to turn left for %f meter" % target_distance)
        send_to(new_loc[0], new_loc[1], loc[2])

def colorDetection(frame, lowerLimit, upperLimit):
    global x_center, y_center, verified
    verified.x = verified.y = None
    
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
            # print("radius: ",radius)
            cv2.circle(frame, (int(x), int(y)), int(radius), color_yellow, 2)
            cv2.circle(frame, center, 5, color_red, -1) # draw the circle and centroid on the frame
            #print("x: ", round(x), "y: ", round(y))
            cv2.putText(frame, f"x: {round(x)}, y: {round(y)}", (int(x - radius), int(y - radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            verified.x = int(cam_width/2-x) #Jarak QR relative terhadap tengah Frame (sumbu x)
            verified.y = int(cam_height/2-y) #Jarak QR relative terhadap tengah Frame (sumbu y)

            # print("[INFO] Object Center coordinates at X0 = {0} and Y0 =  {1}".format(x, y))
            # print("X error: ", verified.x)
            # print("Y error: ", verified.y)
        
    return frame

def cam(mission):
    lowerLimit = None
    upperLimit = None

    if mission == 1:
        lowerLimit = np.array([10, 100, 20])  # Surprisingly, this is more stable than the previous one lol
        upperLimit = np.array([25, 255, 255])
    elif mission == 2:
        lowerLimit = np.array([170, 50, 50])  # THIS IS THE ONE
        upperLimit = np.array([180, 255, 255])

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    global cam_width, cam_height
    cam_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))   #Get Lebar FRAME
    cam_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) #Get Tinggi FRAME

    while True:
        ret, frame = cap.read()  # Read current frame
        if not ret:
            break

        # Process the frame for object detection
        processed_frame = colorDetection(frame, lowerLimit, upperLimit)

        # Show the processed frame
        center_x, center_y = int(cam_width/2), int(cam_height/2)
        cv2.circle(processed_frame, (center_x, center_y), 100, color_green, 2)
        cv2.putText(processed_frame, "Frame: "+ str(cam_width) + "x" + str(cam_height), (10,20), font, 0.5, color_white, 1)
        cv2.imshow("Object Detection", processed_frame)

        if cv2.waitKey(1) and verified.confirmed:
            break

    cap.release()
    cv2.destroyAllWindows()

def get_distance(TRIG, ECHO):
    # Send a 10us pulse to trigger the sensor
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Measure the pulse duration on the ECHO pin
    pulse_start = time.time()
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    pulse_end = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    # Speed of sound at sea level is approximately 343 meters per second
    # Distance = (time taken by pulse to return * speed of sound) / 2
    distance = (pulse_duration * 34300) / 2

    return distance

def readSensor():
    try:
        while True:
            # Get distances from all sensors
            sensor.right = get_distance(right_TRIG, right_ECHO)
            sensor.left = get_distance(left_TRIG, left_ECHO)
            sensor.forward = get_distance(forward_TRIG, forward_ECHO)
            #down_dist = get_distance(down_TRIG, down_ECHO)
            time.sleep(0.1)  # Set a delay of 0.1 seconds between measurements
        
            return sensor.right, sensor.left, sensor.forward
    
    except KeyboardInterrupt:
        GPIO.cleanup()

def magnet_turun():
    GPIO.output(motor_pin1, GPIO.HIGH)
    GPIO.output(motor_pin2, GPIO.LOW)
    time.sleep(5)
    stop()

def magnet_naik():
    GPIO.output(motor_pin1, GPIO.LOW)
    GPIO.output(motor_pin2, GPIO.HIGH)
    time.sleep(5)
    stop()

def stop():
    GPIO.output(motor_pin1, GPIO.LOW)
    GPIO.output(motor_pin2, GPIO.LOW)
    time.sleep(1)

def deactiveMagnet():
    GPIO.output(magnet_PIN, 1)

def align():
    try:
        print("Mode: "+vehicle.mode.name)
        vehicle.channels.overrides['3'] = 1500
        vehicle.mode = VehicleMode('LOITER')
        print("Change to LOITER mode")
        time.sleep(1)
        print("Mode: "+vehicle.mode.name)
        print("Emulating Joystick Control...")
        while verified.confirmed is False:
            print(verified.confirmed)
            if verified.x and verified.y:
                x, y=verified.x , verified.y
                r, l, f=sensor.right, sensor.left, sensor.forward
                print(str(x)+','+str(y))
                if x<20 and x>-20 and y<20 and y>-20:
                    # vehicle.channels.overrides = {}
                    # vehicle.mode = VehicleMode('GUIDED')
                    # set_roi(location=vehicle.home_location)
                    vehicle.channels.overrides['7']=1800
                    print("Tunggu bentar...")
                    time.sleep(1)
                    verified.confirmed=True
                    print("Deteksi selesai!!!")
                    time.sleep(2)
                    break
                elif x>20: #kiri
                    vehicle.channels.overrides['1']=1300
                    print('kiri')
                elif x<-20: #kanan
                    vehicle.channels.overrides['1']=1700
                    print('kanan')
                elif y>20: #mundur
                    vehicle.channels.overrides['2']=1300
                    print('mundur')
                elif y<-20: #maju
                    vehicle.channels.overrides['2']=1700            
                    print('maju')
                vehicle.flush()
                time.sleep(2)
            else:
                # if there's no object
                print("Looking for object...")
                vehicle.channels.overrides = {}
                # if sensor depan ada tembok jarak 20-60 cm
                if f <= targetDistanceFront and f > 20:
                    # jarak kanan lebih besar dari 60 cm
                    if r > targetDistanceSide:
                        vehicle.channels.overrides['1']=1700
                        print('kanan')
                    # jarak kiri lebih besar dari 60 cm
                    elif l > targetDistanceSide:
                        vehicle.channels.overrides['1']=1300
                        print('kiri')
                else:
                    # obstacle avoidance
                    if r > targetDistanceSide and l < targetDistanceSide:
                        vehicle.channels.overrides['1']=1700
                        print('kanan')
                    elif r < targetDistanceSide and l > targetDistanceSide:
                        vehicle.channels.overrides['1']=1300
                        print('kiri')
                    else:
                        vehicle.channels.overrides['2'] = 1700
                        print("maju")
                time.sleep(2)
    except:
        print("ERROR.")
        vehicle.channels.overrides = {}
        vehicle.channels.overrides['3'] = 1500

############################# MAIN FUNCTION #############################

if __name__ == "__main__":
    try:
        # Arm and takeoff
        arm_and_takeoff(0.7)
        print("Preparing the mission")
        time.sleep(10)
        print("Mission start!!!")

        ################################ ORANGE MISSION ################################

        # Start the camera and 3 sensors thread
        print("Start ORANGE mission")
        camera_thread = threading.Thread(target=cam, args=(1,))
        sensor_thread = threading.Thread(target=sensor)
        camera_thread.daemon = True  # Daemonize the thread so it stops when the main program exits
        sensor_thread.daemon = True
        camera_thread.start()
        sensor_thread.start()
        time.sleep(5)

        # Main loop for the mission
        while not verified.confirmed:
            # Perform align 
            align()
            # Add a delay to avoid excessive CPU usage
            time.sleep(1)

        # Stop the camera thread
        print("Drop the magnet!!!!")
        magnet_turun()
        time.sleep(5)
        print("Object attached")
        magnet_naik()
        verified.confirmed = True  # Set this flag to signal the camera thread to stop
        camera_thread.join()  # Wait for the camera thread to complete  

        ################################ RED MISSION ################################

        verified.confirmed = False
        # start camera thread for red mission
        print("Start RED mission")
        camera_thread = threading.Thread(target=cam, args=(2,))
        camera_thread.daemon = True  # Daemonize the thread so it stops when the main program exits
        camera_thread.start()
        time.sleep(5)

        # Main loop for the mission
        while not verified.confirmed:
            # Perform align
            align()
            # Add a delay to avoid excessive CPU usage
            time.sleep(1)

        # Stop the camera thread
        print("DROPPING the object!!!")
        deactiveMagnet()
        time.sleep(5)
        verified.confirmed = True  # Set this flag to signal the camera thread to stop
        camera_thread.join()    # Wait for the camera thread to complete
        sensor_thread.join()    # stop sensor threading

        ################################ LANDING ################################
        print("Change to GUIDED mode")
        vehicle.mode = VehicleMode('GUIDED')
        print("Get ready to landing")
        navigation("FORWARD", 2)
        # Land the vehicle
        print("Landing.....")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(5)

        # Close the vehicle connection
        vehicle.close()

        print("Mission complete")

    except KeyboardInterrupt:
        print("Keyboard Interrupted")
        print("Emergency Landing.....")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(5)
        print("Close vehicle")
        vehicle.close()