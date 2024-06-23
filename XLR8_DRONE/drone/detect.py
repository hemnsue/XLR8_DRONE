import jetson.inference
import jetson.utils
import cv2

# Load the object detection model
net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)

# Load input image
image_path = 'input_image.jpg'  # Path to the input image
image = cv2.imread(image_path)
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# Convert image to CUDA format
cuda_img = jetson.utils.cudaFromNumpy(image)

# Perform inference
detections = net.Detect(cuda_img, overlay="box,labels,conf")

# Visualize results
for detection in detections:
    left = int(detection.Left)
    top = int(detection.Top)
    right = int(detection.Right)
    bottom = int(detection.Bottom)
    class_id = detection.ClassID
    class_name = net.GetClassDesc(class_id)

    cv2.rectangle(image, (left, top), (right, bottom), (0, 255, 0), 2)
    cv2.putText(image, '{}: {:.2f}'.format(class_name, detection.Confidence), (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# Display the image with detections
cv2.imshow('Object Detection', cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
cv2.waitKey(0)
cv2.destroyAllWindows()

