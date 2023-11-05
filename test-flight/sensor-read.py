import RPi.GPIO as GPIO
import time

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Define the TRIG and ECHO pins for each sensor
right_TRIG = 17
right_ECHO = 18

left_TRIG = 27
left_ECHO = 23

forward_TRIG = 22
forward_ECHO = 24

down_TRIG = 10
down_ECHO = 25

# Set up the GPIO pins
GPIO.setup(right_TRIG, GPIO.OUT)
GPIO.setup(right_ECHO, GPIO.IN)

GPIO.setup(left_TRIG, GPIO.OUT)
GPIO.setup(left_ECHO, GPIO.IN)

GPIO.setup(forward_TRIG, GPIO.OUT)
GPIO.setup(forward_ECHO, GPIO.IN)

GPIO.setup(down_TRIG, GPIO.OUT)
GPIO.setup(down_ECHO, GPIO.IN)

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

def output():
    right_dist = get_distance(right_TRIG, right_ECHO)
    left_dist = get_distance(left_TRIG, left_ECHO)
    forward_dist = get_distance(forward_TRIG, forward_ECHO)
    down_dist = get_distance(down_TRIG, down_ECHO)

    return right_dist, left_dist, forward_dist, down_dist


try:
    while True:
        # Get distances from all sensors
        right_dist = get_distance(right_TRIG, right_ECHO)
        left_dist = get_distance(left_TRIG, left_ECHO)
        forward_dist = get_distance(forward_TRIG, forward_ECHO)
        down_dist = get_distance(down_TRIG, down_ECHO)

        print(f"Right Distance: {right_dist:.2f} cm")
        print(f"Left Distance: {left_dist:.2f} cm")
        print(f"Forward Distance: {forward_dist:.2f} cm")
        print(f"Down Distance: {down_dist:.2f} cm")

        time.sleep(0.1)  # Set a delay of 0.1 seconds between measurements

except KeyboardInterrupt:
    GPIO.cleanup()
