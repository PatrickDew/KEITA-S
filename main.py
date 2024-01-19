#!/usr/bin/env pybricks-micropython
from camera import Camera
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
import time
import math

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Play a sound.
ev3.speaker.beep()

test_motorA = Motor(Port.A)
test_motorB = Motor(Port.B)

model_directory = '/home/patricknuttha/Documents/From Windows/Ubuntu-20240118T103340Z-001/Teambased/Teambased Invention of Engineering-20240119T060112Z-001/Teambased Invention of Engineering/train17/weights/best.pt'
camera = Camera(model_directory)

# Distance threshold for obstacle avoidance (adjust as needed)
distance_threshold = 0.5  # in meters
initial_angle = 0

def main():
    while True:
        # Capture frame from the camera
        W, H = 640, 480

        color_frame, depth_frame = camera.capture_frame()

        # Process the frame using YOLO and obstacle detection
        obstacle_data_list = camera.process_frame(color_frame, depth_frame)

        for obstacle_data in obstacle_data_list:
            obstacle_detected = obstacle_data['depth_value'] < distance_threshold

            if obstacle_detected:
                center_x = obstacle_data['center_x']
                center_y = obstacle_data['center_y']

                angle_to_obstacle = math.atan2(W/2 - center_x, H/2 - center_y) * (180 / math.pi)
                adjusted_angle = angle_to_obstacle - initial_angle

                if -90 <= adjusted_angle <= 0:
                    rotation_degree = adjusted_angle
                    #print(f"Obstacle detected! Rotating to the left {rotation_degree} degrees...")

                    speed_factor = 1 + abs(rotation_degree) / 90
                    test_motorA.run(-300 * speed_factor)
                    test_motorB.run(300 * speed_factor)
                    time.sleep(2)
                    test_motorA.stop()
                    test_motorB.stop()

                elif 0 < adjusted_angle <= 90:
                    rotation_degree = adjusted_angle
                    #print(f"Obstacle detected! Rotating to the right {rotation_degree} degrees...")

                    speed_factor = 1 + abs(rotation_degree) / 90
                    test_motorA.run(300 * speed_factor)
                    test_motorB.run(-300 * speed_factor)
                    time.sleep(2)
                    test_motorA.stop()
                    test_motorB.stop()

                else:
                    print("Moving forward...")
                    test_motorA.run(500)
                    test_motorB.run(500)
                    time.sleep(2)

        # Other logic as needed
        time.sleep(0.5)

if __name__ == "__main__":
    main()
