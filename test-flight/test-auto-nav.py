from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time, math

# connection = "/dev/ttyS0"
# print("Connecting to vehicle on: %s" % (connection,))
# Connect to the Vehicle
#vehicle = connect('/dev/ttyS0', wait_ready=True, baud=921600) #Konek ke Serial0(Raspberry Pi 4)
#921600 is the baudrate that you have set in the mission plannar or qgc
# vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
# vehicle = connect('COM9', wait_ready=True, baud=57600) #Konek ke port COMx(WINDOWS) via Telemetry
vehicle = connect('tcp:127.0.0.1:5763', wait_ready=True, baud=57600) #konek via TCP(SITL)

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

def change_alt(target, step):
    # This function will increase or decrease the altitude of the vehicle based on the input.
    #target = int(input("Enter your target altitude:\n").upper())
    
    actual_altitude = int(vehicle.location.global_relative_frame.alt)
    changed_altitude = [(actual_altitude + target), (actual_altitude - target)]

    if step == "INC":
        if changed_altitude[0] <= 50:
            send_to(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, changed_altitude[0])
            print(" Reached the new altitude.")
        else:
            print(" Vehicle Reached Maximum Altitude!!!")

    if step == "DEC":
        if changed_altitude[1] >= 0.2:
            send_to(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, changed_altitude[1])
        else:
            print(" Vehicle Reached Minimum Altitude!!!")

def destination_location(homeLattitude, homeLongitude, distance, bearing):
    """

    This function returns the latitude and longitude of the
    destination location, when distance and bearing is provided.

    Inputs:
        1.  homeLattitude       -   Home or Current Location's  Latitude
        2.  homeLongitude       -   Home or Current Location's  Latitude
        3.  distance            -   Distance from the home location
        4.  bearing             -   Bearing angle from the home location

    """

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

def main():
    # Sets the heading angle constant during flight
    if vehicle.parameters["WP_YAW_BEHAVIOR"] != 0:
        vehicle.parameters["WP_YAW_BEHAVIOR"] = 0
        print("Changed the Vehicle's WP_YAW_BEHAVIOR parameter")
    try:
        arm_and_takeoff(0.5)
        time.sleep(5)
        print("Initializing to go forward")
        time.sleep(2)
        print("Going forward for orange mission")
        navigation('FORWARD', 1)
        time.sleep(2)
        # print("Change mode to POSHOLD")
        # vehicle.mode = VehicleMode("POSHOLD")
        print("Orange detection scene")
        time.sleep(10)
        print("Orange mission done")
        time.sleep(2)
        # print("Change mode to GUIDED")
        # vehicle.mode = VehicleMode("GUIDED")
        # time.sleep(2)
        print("Change altitude to 1m")
        change_alt(1, 'INC')
        print("Initializing to go forward")
        time.sleep(2)
        print("Going forward")
        navigation('FORWARD', 3.95)
        time.sleep(5)
        print("Initializing to go left")
        time.sleep(2)
        navigation('LEFT', 4.93)
        time.sleep(7)
        # print("Change mode to POSHOLD")
        # vehicle.mode = VehicleMode("POSHOLD")
        print("Change altitude to 0.5m")
        change_alt(0.5, 'DEC')
        print("Red detection scene")
        time.sleep(10)
        print("Red mission done")
        time.sleep(2)
        # print("Change mode to GUIDED")
        # vehicle.mode = VehicleMode("GUIDED")
        # time.sleep(2)
        print("Initializing to landing")
        time.sleep(2)
        navigation('BACKWARD', 3)
        time.sleep(5)
        print("Landing")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(10)
        print("Mission Complete")
        print("Close vehicle object")
        vehicle.close()
    except KeyboardInterrupt:
        print(" The flight was stopped by user!!!")
        print("Start to landing")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(5)
        print("Close vehicle object")
        vehicle.close()
    except Exception as e:
        print("Error during main(): ", str(e))
        print("Performing emergency landing...")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(10)
        vehicle.close()  # Close vehicle object
        exit(1)
    

if __name__ == "__main__":
    main()