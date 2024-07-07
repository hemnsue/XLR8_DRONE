import jetson.inference
import jetson.utils
import cv2
import math
from pymavlink import mavutil
import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command

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


#vehicle
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

# Set up the camera
video_capture=cv2.VideoCapture("nvarguscamerasrc sensor-id=1 sensor-mode=4 ! "
"video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)60/1 ! "
"nvvidconv flip-method=0 ! "
"video/x-raw, width=(int)640, height=(int)480, format=(string)BGRx ! "
"videoconvert ! "
"video/x-raw, format=(string)BGR ! appsink",cv2.CAP_GSTREAMER)  # using videoSource for camera input

# Process frames from the camera++
use_frame=0
while(True):
    ret,frame_camera_input=video_capture.read()
    j=+1
    if j==42:
        use_frame=frame_camera_input
        cv2.imwrite("test_image_capture_1.jpg",use_frame)
        break
detections = net.Detect(use_frame, 640, 480)

# Print the detections
print("Detections:")
for detection in detections:
    center_x, center_y = detection.Center
    print(f" - Detected object of class '{net.GetClassDesc(detection.ClassID)}', "
          f"confidence {detection.Confidence:.2f}, "
          f"center coordinates ({center_x}, {center_y})")
    centerpt=[]
    centerpt.append(center_x)
    centerpt.append(center_y)
video_capture.release()
#Calculate Distance
disangel=calculate_distance(centerpt)
distance=disangel[0]
heading_angle=disangel[1]
#Find yaw angle and clock/anit clockwise rotation
yaw,clock=find_yaw(heading_angle)
#rotate
conditional_yaw(yaw,clock)
#Calculate the latitude and longitude
find_lat_longitude_target(distance,vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,vehicle.heading)

print(disangel)




