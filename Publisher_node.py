import rospy
from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO
from ultralytics.utils import ASSETS
from ultralytics.models.yolo.detect import DetectionPredictor
import cv2
import time

model = YOLO("yolov8s.pt")
# Initialize ROS node
rospy.init_node('bottle_position_publisher', anonymous=True)

# Create publisher for bottle position
bottle_position_pub = rospy.Publisher('detected_bottle_position', Float32MultiArray, queue_size=10)

# Initialize detected bottle position
detected_bottle_position = Float32MultiArray()
detected_bottle_position.data = [0.0, 0.0, 0.0]  # Initialize to zero coordinates and confidence

while True: 
    # Capture frame from webcam
    cam = cv2.VideoCapture("http://192.168.157.93:8080/video") 
    captured, frame = cam.read() 

    if captured:
        # Perform object detection on the captured frame
        results = model.predict(source = "http://192.168.106.123:8080/video", show = True)
        results = model.predict(frame)
        result = results[0]

        # Reset detected bottle position
        detected_bottle_position.data = [0.0, 0.0, 0.0]

        # Iterate through detected objects
        for box in result.boxes:
            class_id = result.names[box.cls[0].item()]
            if class_id == "bottle":  # Check if object is a bottle
                cords = box.xyxy[0].tolist()
                cords = [round(x) for x in cords]
                x1, y1, x2, y2 = cords  
                center_x = (x1 + x2) / 2  
                center_y = (y1 + y2) / 2  
                conf = round(box.conf[0].item(), 2)

                # Update detected bottle position
                detected_bottle_position.data = [center_x, center_y, conf]
                break  # Exit the loop after detecting the first bottle

        # Publish the detected bottle position
        bottle_position_pub.publish(detected_bottle_position)

        # Print the position of the detected bottle
        print("Object type: bottle")
        print("Center coordinates:", detected_bottle_position.data[:2])
        print("---------------------------------------")

    time.sleep(2)  # Add a delay before capturing the next frame

# Spin ROS no
de
rospy.spin()
