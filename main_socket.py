#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
import time
import math
import socket
import pickle

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Play a sound.
ev3.speaker.beep()

test_motorA = Motor(Port.A)
test_motorB = Motor(Port.B)

model_directory = '/home/patricknuttha/Documents/From Windows/Ubuntu-20240118T103340Z-001/Teambased/Teambased Invention of Engineering-20240119T060112Z-001/Teambased Invention of Engineering/train17/weights/best.pt'

# Distance threshold for obstacle avoidance (adjust as needed)
distance_threshold = 0.5  # in meters
initial_angle = 0

def receive_obstacle_data(host, port):
    # Create a socket to listen for incoming data
    W, H = 640, 480
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()

        print("Waiting for connection...")
        conn, addr = s.accept()

        with conn:
            #print(f"Connected by {addr}")

            while True:
                # Receive and deserialize the obstacle data
                data = conn.recv(4096)
                if not data:
                    break
                
                obstacle_data_list = pickle.loads(data)

                # Process obstacle data and control the robot
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

# Run the server on the EV3
while True:
    receive_obstacle_data('169.254.70.63', 5555)  # Use the EV3's IP address