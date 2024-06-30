import cv2 as cv
import numpy as np

# Load your custom YOLO model
net = cv.dnn.readNetFromDarknet("custom_yolov3.cfg", "custom_yolov3.weights")
net.setPreferableBackend(cv.dnn.DNN_BACKEND_CUDA)
net.setPreferableTarget(cv.dnn.DNN_TARGET_CUDA)

# Load your custom class names
with open("custom_classes.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

# Load an image
image = cv.imread("test.jpg")
original_height, original_width = image.shape[:2]

# Resize the image to 416x416 (or another size that matches your model)
input_size = 416  # Change to 320, 608, etc. if your model is trained with a different size
blob = cv.dnn.blobFromImage(cv.resize(image, (input_size, input_size)), 0.00392, (input_size, input_size), (0, 0, 0), True, crop=False)
net.setInput(blob)
outs = net.forward(output_layers)

# Initialize lists to hold class IDs, confidences, and bounding boxes
class_ids = []
confidences = []
boxes = []

# Process each detection
for out in outs:
    for detection in out:
        scores = detection[5:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]
        if confidence > 0.5:
            # Object detected
            center_x = int(detection[0] * input_size)
            center_y = int(detection[1] * input_size)
            w = int(detection[2] * input_size)
            h = int(detection[3] * input_size)
            x = int(center_x - w / 2)
            y = int(center_y - h / 2)

            # Scale bounding box coordinates back to the original image size
            x = int(x * (original_width / input_size))
            y = int(y * (original_height / input_size))
            w = int(w * (original_width / input_size))
            h = int(h * (original_height / input_size))

            boxes.append([x, y, w, h])
            confidences.append(float(confidence))
            class_ids.append(class_id)

# Perform non-max suppression
indices = cv.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

# Draw bounding boxes on the original image
for i in indices:
    i = i[0]
    box = boxes[i]
    x, y, w, h = box[0], box[1], box[2], box[3]
    label = str(classes[class_ids[i]])
    confidence = confidences[i]
    color = (0, 255, 0)
    cv.rectangle(image, (x, y), (x + w, y + h), color, 2)
    cv.putText(image, label, (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

# Display the output image
cv.imshow("Image", image)
cv.waitKey(0)
cv.destroyAllWindows()

