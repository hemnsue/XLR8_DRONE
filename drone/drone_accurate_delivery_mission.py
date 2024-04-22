from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
import argparse  
##LOAD THE OBJECT DETECTIONS MODULES
import cv2
import jetson.inference
import jetson.utils
net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)

def arm_and_takeoff(aTargetAltitude,i):

    print("\nSet Vehicle.mode = GUIDED (currently: %s)" % vehicle.mode.name) 
    vehicle.mode = VehicleMode("GUIDED")
    
    while not vehicle.mode.name=='GUIDED':  #Wait until mode has changed
        print(" Waiting for mode change ...")
        time.sleep(1)

    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)
    if i==1:
        desired_angle = 0
        channel=10
        control_servo_angle(channel, desired_angle)

##Simple go to the required gps location command
def go_to_required_location(targetlocation):
    vehicle.simple_goto(targetlocation)
    while True:
        if targetlocation.distance_to(vehicle.location.global_relative_frame) <= 0.25:
            print("Reached Target Location")
            break
        time.sleep(1)

#Function to control the servo motor
def control_servo_angle(channel, angle):
    pwm_range = (2000 - 1000)  # PWM range (maximum - minimum)
    angle_range = 180  # Maximum angle
    pwm_per_degree = pwm_range / angle_range
    pwm_value = int(1000 + angle * pwm_per_degree)
    vehicle.channels.overrides[channel] = pwm_value
    print(f"Set PWM value for channel {channel} to achieve {angle} degrees: {pwm_value}")

##If we want to increase or decrese the height of the drone depending on the on object detected on the same latitude and longitude
def increase_decrease_height_same_latitude_and_longitude(choice,i):
    current_altitude=vehicle.location.global_relative_frame.alt
    ##If choice=1 increase the height by 2m and if choice=2 decrease the height by 1m or according to the condition
    if choice==1:
        new_alt=current_altitude+2
        vehicle.simple_goto(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,new_alt)
        while True:
            current_altitude = vehicle.location.global_relative_frame.alt
            if current_altitude >= new_alt * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)
    elif choice==2:
        if current_altitude<=0.25 and i==0:
        ###Proceed for landing
            vehicle.mode=VehicleMode("LAND")
            while vehicle.armed:
                print("Waiting for landing")
                time.sleep(1)

            channel = '10'
            desired_angle = 110
            control_servo_angle(channel, desired_angle)

            vehicle.armed=False

        elif current_altitude<=0.25 and i==1:
        ###Proceed for landing
            vehicle.mode=VehicleMode("LAND")
            while vehicle.armed:
                print("Waiting for landing")
                time.sleep(1)
            vehicle.armed=False

        ###Heights in low height range   
        elif current_altitude>0.25 and current_altitude<=1:
            vehicle.simple_goto(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,0.25)
            while True:
                current_altitude = vehicle.location.global_relative_frame.alt
                if current_altitude <= new_alt * 1.05:
                    print("Reached target altitude")
                    break
                time.sleep(1)
        ###Heights in high range 
        else:
            new_alt=current_altitude-1
            vehicle.simple_goto(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,new_alt)
            while True:
                current_altitude = vehicle.location.global_relative_frame.alt
                if current_altitude <= new_alt * 1.05:
                    print("Reached target altitude")
                    break
                time.sleep(1)

def calculate_distance():
    return distance


parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect',help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
go_to_location=args.gps_location.split(" ")
connection_string = args.connect
sitl = None

#Start SITL if no connection string specified

if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

#vehicle home_location, mode, and armed attributes (the only settable attributes)
#GETTING HOME LOCATION OF THE DRONE

while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    if not vehicle.home_location:
        print(" Waiting for home location ...")

# We have a home location, so print it!        

print("\n Home location: %s" % vehicle.home_location)
homelocation=[vehicle.home_location.lat,vehicle.home_location.lon,vehicle.home_location.alt]
# From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).

video_capture=cv2.VideoCapture("nvarguscamerasrc sensor-id=1 sensor-mode=4 ! "
"video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)60/1 ! "
"nvvidconv flip-method=0 ! "
"video/x-raw, width=(int)640, height=(int)480, format=(string)BGRx ! "
"videoconvert ! "
"video/x-raw, format=(string)BGR ! appsink",cv2.CAP_GSTREAMER)

for i in range(2):

    #TAKING OFF
    arm_and_takeoff(10,i)

    #MOVING TO THE REQUIRED LOCATION
    if i==0:
        target_location=LocationGlobalRelative(go_to_location[0],go_to_location[1],go_to_location[2])
    elif i==1:
        target_location=LocationGlobalRelative(homelocation[0],homelocation[1],homelocation[2])

    go_to_required_location(target_location)

    ##Detection of object in the area

    while True:
    	
        j=0
        use_frame=0
        #Taking camera input
        while(True):
            ret,frame_camera_input=video_capture.read()
            j=+1
            if j==42:
                use_frame=frame_camera_input
                break
        #use_frame is the camera frame to be used for inference    
        use_frame = cv2.cvtColor(use_frame, cv2.COLOR_BGR2RGB)
        # Convert image to CUDA format
        cuda_img = jetson.utils.cudaFromNumpy(use_frame)
        # Perform inference
        detections = net.Detect(cuda_img, overlay="box,labels,conf")
        if detections==None:
            object=False
            ##If object not present in the given target coordinates
            ##Increase Height by 2m
            increase_decrease_height_same_latitude_and_longitude(1,i)

        else:
            #Passing the camera input to the model
            for detection in detections:
                left = int(detection.Left)
                top = int(detection.Top)
                right = int(detection.Right)
                bottom = int(detection.Bottom)
                class_id = detection.ClassID
                class_name = net.GetClassDesc(class_id)
        ##If object present in the frame,decrease the height by 1m after moving on top of the object after finding the gps coordinates of the object
            distance=calculate_distance()
            increase_decrease_height_same_latitude_and_longitude(2,i)