from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
import time
import math
from pymavlink import mavutil
import argparse  
##LOAD THE OBJECT DETECTIONS MODULES
import cv2
import jetson.inference
import jetson.utils
# Define paths and parameters directly
MODEL_PATH = "path/to/model.engine"
LABELS_PATH = "path/to/labels.txt"
INPUT_BLOB_NAME = "input"
OUTPUT_CVG_NAME = "scores"
OUTPUT_BBOX_NAME = "boxes"
CAMERA_DEVICE = "/dev/video0"

# Load the custom SSD MobileNet v2 model
net = jetson.inference.detectNet(
    argv=[
        '--model=' + MODEL_PATH,
        '--labels=' + LABELS_PATH,
        '--input-blob=' + INPUT_BLOB_NAME,
        '--output-cvg=' + OUTPUT_CVG_NAME,
        '--output-bbox=' + OUTPUT_BBOX_NAME
    ]
)

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

#Calculate the centerpoint and angel of the point with respect to 0degree
def calculate_distance(centerpt):
    distance=math.sqrt((320-centerpt[0])**2+(240-centerpt[1])**2)
    loc=[]
    loc.append(distance)
    angle=0
    if centerpt[0]==320:
        if centerpt[1]>240:
            angle=90
        elif centerpt[1]<240:
            angle=-90
        else:
            angle=0.0
    else:
        angle=math.degrees(math.atan2(centerpt[1]-240,centerpt[0]-320))
    
    #Normalise angle to be within -180 to 180
    #angle=(angle+180)%360-180
    loc.append(angle)
    return loc

#Find yaw angel according to the heading of drone
def find_yaw(heading_angle):
    yaw=0
    clock=1
    if heading_angle<90:
        yaw=90-heading_angle
    elif heading_angle>90 and heading_angle<180:
        yaw=heading_angle-90
        clock=-1
    elif heading_angle>180 and heading_angle<270:
        yaw=heading_angle-90
        clock=-1
    elif heading_angle>270 and heading_angle<360:
        yaw=360-heading_angle+90
    elif heading_angle==90:
        yaw=0
    elif heading_angle==180:
        yaw=90
        clock=-1
    elif heading_angle==270:
        yaw=180
        clock=-1
    elif heading_angle==0 or heading_angle==360:
        yaw=90
    return yaw,clock

#matching the drones heading angel and the angel of object detection, resulting in same heading for both
def conditional_yaw(yaw,clock):
    # Create the CONDITION_YAW command using command_long_encode()
    is_relative=1
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        yaw,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        1,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used

    # Send the command to the vehicle
    vehicle.send_mavlink(msg)

def find_yaxis_coordinates(distance):
    yaxis_coord=distance
    return yaxis_coord

def find_real_life_distance(yaxis_coord):
    #Since height to distance ratio is constant
    ratio=1.77
    distance_in_frame=(vehicle.location.global_relative_frame.alt/ratio)/2
    distance_found=yaxis_coord*distance_in_frame/240
    return distance_found


#Finding the new latitude and longitude
def find_lat_longitude_target(distance,lat1,lon1,heading_angle):
    # Earth's radius in kilometers
    radius_earth = 6371.0
    
    # Convert distance to radians
    angular_distance = distance / radius_earth
    
    # Convert bearing to radians
    bearing_rad = math.radians(heading_angle)
    
    # Convert latitude and longitude from degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    
    # Calculate the destination coordinates in radians
    lat2_rad = math.asin(math.sin(lat1_rad) * math.cos(angular_distance) +
                         math.cos(lat1_rad) * math.sin(angular_distance) * math.cos(bearing_rad))
    
    lon2_rad = lon1_rad + math.atan2(math.sin(bearing_rad) * math.sin(angular_distance) * math.cos(lat1_rad),
                                     math.cos(angular_distance) - math.sin(lat1_rad) * math.sin(lat2_rad))
    
    # Convert destination coordinates back to degrees
    lat2 = math.degrees(lat2_rad)
    lon2 = math.degrees(lon2_rad)
    
    return lat2, lon2




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

    while vehicle.armed:
    	
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
            left=right=top=bottom=0

            #Passing the camera input to the model
            for detection in detections:
                left = int(detection.Left)
                top = int(detection.Top)
                right = int(detection.Right)
                bottom = int(detection.Bottom)
                class_id = detection.ClassID
                class_name = net.GetClassDesc(class_id)


            centerpt=[int(top+bottom)/2,int(left+right)/2]

            '''for detection in detections:
            center_x, center_y = detection.Center'''
        ##If object present in the frame,decrease the height by 1m after moving on top of the object after finding the gps coordinates of the object
        ##Calculate distance 
            disangel=calculate_distance(centerpt)
            distance=disangel[0]
            heading_angle=disangel[1]
        #Using heading angle and the distance calculated find the new geo cordinates
        #yaw rotaion for heading matching
            yaw,clock=find_yaw(heading_angle)
            conditional_yaw(yaw,clock)
        #find y axis coordinates
            yaxis_coord=find_yaxis_coordinates(distance)
        #find the real distance
            distance=find_real_life_distance(yaxis_coord)
        #Calculate the latitude and longitude
            lat,lon=find_lat_longitude_target(distance,vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,vehicle.heading)
            target_location=LocationGlobalRelative(lat,lon,vehicle.location.global_relative_frame.alt)
        #move to new lat,lon
            go_to_required_location(target_location)
        #decrease height
            increase_decrease_height_same_latitude_and_longitude(2,i)