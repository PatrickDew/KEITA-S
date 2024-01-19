import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import math
import socket
import pickle

# Initialize camera and YOLO model here
model_directory = '/home/patricknuttha/Documents/From Windows/Ubuntu-20240118T103340Z-001/Teambased/Teambased Invention of Engineering-20240119T060112Z-001/Teambased Invention of Engineering/train17/weights/best.pt'
model = YOLO(model_directory)
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
profile = pipeline.start(config)
align_to = rs.stream.color
align = rs.align(align_to)

def capture_frame():
    # Capture a frame from the camera
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()

    color_image = np.asanyarray(color_frame.get_data())
    # Display the color frame
    display_frame(color_image)

    return color_image, depth_frame

def process_frame(color_frame, depth_frame):
    # Process the frame using the YOLO model and obstacle detection logic
    results = model.predict(
        source=color_frame,
        conf=0.5,
        iou=0.75
    )

    obstacle_data_list = []

    for r in results:
        for box in r.boxes:
            center_x = int((box.xyxy[0, 0] + box.xyxy[0, 2]) / 2)
            center_y = int((box.xyxy[0, 1] + box.xyxy[0, 3]) / 2)
            depth_value = depth_frame.get_distance(center_x, center_y)

            obstacle_data = {
                'center_x': center_x,
                'center_y': center_y,
                'depth_value': depth_value,
            }

            obstacle_data_list.append(obstacle_data)

    return obstacle_data_list





def send_obstacle_data(obstacle_data_list, host, port):
    # Create a socket connection
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))

        # Serialize and send the obstacle data
        serialized_data = pickle.dumps(obstacle_data_list)
        s.sendall(serialized_data)




        

def display_frame(color_frame):
    # Display the color frame using OpenCV
    cv2.imshow('RealSense Camera', color_frame)
    cv2.waitKey(1)

# Set your host and port for socket communication
host = '169.254.70.63'
port = 5555

# Main loop
while True:
    color_frame, depth_frame = capture_frame()
    obstacle_data_list = process_frame(color_frame, depth_frame)
    send_obstacle_data(obstacle_data_list, host, port)
