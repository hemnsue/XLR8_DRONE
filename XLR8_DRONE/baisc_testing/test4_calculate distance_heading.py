import jetson.inference
import jetson.utils
import cv2
import math

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
disangel=calculate_distance(centerpt)
distance=disangel[0]
heading_angle=disangel[1]

print(disangel)




