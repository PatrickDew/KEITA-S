# camera.py
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import math

class Camera:
    def __init__(self, model_directory):
        # Initialize camera and YOLO model here
        self.model = YOLO(model_directory)
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.profile = self.pipeline.start(self.config)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

    def capture_frame(self):
        # Capture a frame from the camera
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        color_image = np.asanyarray(color_frame.get_data())
        return color_image, depth_frame

    def process_frame(self, color_frame, depth_frame):
        # Process the frame using the YOLO model and obstacle detection logic
        results = self.model.predict(
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
