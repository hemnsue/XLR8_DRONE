import jetson.inference
import jetson.utils
import cv2
import matplotlib.pyplot as plt

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
    
plt.imshow(use_frame)
plt.show()
video_capture.release()

