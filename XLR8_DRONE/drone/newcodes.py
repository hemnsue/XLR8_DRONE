import cv2
from flask import Flask
from flask import render_template,Response,stream_with_context,request
import numpy

#importing libraries required
#collecting frames from camera
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=320,
    display_height=240,
    framerate=60,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d sensor-mode=4 ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

#showing output
def show_camera():

    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    print(gstreamer_pipeline(flip_method=0))
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    if video_capture.isOpened():
        try:

            while True:
                ret_val, frame = video_capture.read()
                # Check to see if the user closed the window
                # Under GTK+ (Jetson Default), WND_PROP_VISIBLE does not work correctly. Under Qt it does
                # GTK - Substitute WND_PROP_AUTOSIZE to detect if window has been closed by user
                if not ret_val:
                    break
                else:
                    ret,buffer=cv2.imencode(".jpeg",frame)
                    frame=buffer.tobytes()
                    yield(b'--frame\r\n' b'Content-type: image/jpeg\r\n\r\n' + frame +b'\r\n')


        finally:
            video_capture.release()
            cv2.destroyAllWindows()
    else:
        print("Error: Unable to open camera")


app=Flask("__name__")

@app.route('/camera')
def camera():
    return render_template('camera.html')

@app.route("/video_feed")
def video_feed():
    return Response(show_camera(),mimetype="multipart/x-mixed-replace; boundary=frame")
app.run(host="0.0.0.0",port="5000",debug=False)

