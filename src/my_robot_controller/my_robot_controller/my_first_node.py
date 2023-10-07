#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np

class MyNode(Node):
    def __init__(self):
        super().__init__("first_node")
        self.tracker()

    def tracker(self):
        # Initialize the video capture object
        vid = cv2.VideoCapture(0)

        # Initialize variables for ball tracking
        previous_frame = None

        while True:
            # Read a frame from the camera
            ret, frame = vid.read()

            # Convert the frame to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Apply Gaussian blur to reduce noise and improve circle detection
            blurred = cv2.GaussianBlur(gray, (17, 17), 2)

            # Use the Hough Circle Transform to detect circles with adaptive parameters
            if previous_frame is not None:
                circles = cv2.HoughCircles(
                    blurred,
                    cv2.HOUGH_GRADIENT,
                    dp=1,
                    minDist=150,
                    param1=50,
                    param2=50,
                    minRadius=50,
                    maxRadius=100
                )

                # If circles are found, draw them and display coordinates
                if circles is not None:
                    circles = np.uint16(np.around(circles))
                    for circle in circles[0, :]:
                        center = (circle[0], circle[1])  # Circle center coordinates
                        radius = circle[2]               # Circle radius

                        # Draw the circle outline
                        cv2.circle(frame, center, radius, (0, 255, 0), 2)

                        # Calculate normalized coordinates with the center as the origin
                        width, height = frame.shape[1], frame.shape[0]
                        normalized_x = (center[0] - width // 2) / (width // 2)
                        normalized_y = (height // 2 - center[1]) / (height // 2)
                        self.get_logger().info(f'{normalized_x, normalized_y}')

                        # Display the normalized coordinates
                        cv2.putText(frame, f"({normalized_x:.2f}, {normalized_y:.2f})", (center[0], center[1] - 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

            # Store the current frame for tracking in the next iteration
            previous_frame = blurred

            # Draw the Cartesian axes
            center_x, center_y = frame.shape[1] // 2, frame.shape[0] // 2
            cv2.line(frame, (center_x, 0), (center_x, frame.shape[0]), (255, 0, 0), 1)  # Vertical line (x-axis)
            cv2.line(frame, (0, center_y), (frame.shape[1], center_y), (0, 255, 0), 1)  # Horizontal line (y-axis)

            # Display the frame with detected circles, axes, and coordinates
            cv2.imshow("Output", frame)

            # Break the loop if 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Release the video capture object and close all windows
        vid.release()
        cv2.destroyAllWindows()
        self.get_logger().info("Hello")

def main(args=None):
    rclpy.init(args=args)  # Start ROS 2 communications

    node = MyNode()
    rclpy.spin(node=node)  # Keep running until manually stopped

    rclpy.shutdown()  # Shutdown ROS 2 communications

if __name__ == "__main__":
    main()
