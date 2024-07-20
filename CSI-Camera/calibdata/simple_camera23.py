
import cv2
import time
import pandas as pd
# To flip the image, modify the flip_method parameter (0 and 2 are the most common)
video_capture = cv2.VideoCapture("nvarguscamerasrc sensor-id= 0 ! "
        "video/x-raw(memory:NVMM), width=(int)1640, height=(int)1232, framerate=(fraction)30/1 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, width=(int)640, height=(int)480, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink", cv2.CAP_GSTREAMER)
k=[]
j=0
l=time.time()
d=[]
paths1=[]
while(True):
        ret,frame_camera_input=video_capture.read()
        if j<42:
                j+=1
        else:
                time.sleep(1)
                k.append(frame_camera_input)
                if time.time()-l>450:
                        break
for i in range(len(k)):
        name=str(i)+".jpg"
        path="/home/xlr8wins/xlr8-drone/CSI-Camera/calibdata/testing/"+name
        d.append(name)
        paths1.append(path)
        cv2.imwrite("testing/"+name,k[i])
j=pd.DataFrame({"name":d,"path":paths1})
j.to_csv("Ourdata.csv")

        


