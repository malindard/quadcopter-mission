# READ YOUR NOTES ON YOUR PHONE FIRST PLS
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# Connect to the Vehicle
vehicle = connect('/dev/ttyS0', wait_ready=True, baud=921600) #Konek ke Serial0(Raspberry Pi 4)
#921600 is the baudrate that you have set in the mission plannar or qgc
# vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
# vehicle = connect('COM9', wait_ready=True, baud=57600) #Konek ke port COMx(WINDOWS) via Telemetry
#vehicle = connect('tcp:127.0.0.1:5763', wait_ready=True, baud=57600) #konek via TCP(SITL)

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

  print("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

  print("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt) 
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print("Reached target altitude")
      break
    time.sleep(1)

# Initialize the takeoff sequence to 15m
arm_and_takeoff(0.7)

print("Take off complete")

# Hover for 10 seconds
time.sleep(10)

print("Now let's land")
vehicle.mode = VehicleMode("LAND")

# Wait for a brief moment to allow the drone to land before closing the vehicle
time.sleep(5)

# Close vehicle object
vehicle.close()