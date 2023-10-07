# Ball Tracking ROS Node

This ROS node is designed to track and visualize the movement of a ball in real-time using computer vision techniques. The node captures video from a camera source (usually the default webcam, but you can change the source by modifying the `cv2.VideoCapture` parameter) and processes the frames to detect and track the ball.

## Prerequisites

Before running this ROS node, make sure you have the following dependencies installed:

- ROS 2: This code is written for ROS 2 (Robot Operating System 2). You must have a ROS 2 installation on your system. Follow the ROS 2 installation instructions for your operating system [here](https://docs.ros.org/en/rolling/Installation.html).

- OpenCV: This code relies on the OpenCV library for computer vision tasks. You can install OpenCV for Python using pip:

  ```
  pip install opencv-python
  ```

## Running the Node

To run the node, follow these steps:

1. Clone or download this repository to your ROS 2 workspace.

2. Build your ROS 2 workspace to compile the node:

   ```
   colcon build
   ```

3. Source your ROS 2 workspace:

   ```
   source install/setup.bash
   ```

4. Run the ROS 2 node:

   ```
   ros2 run <your_package_name> <your_node_name>
   ```

   Replace `<your_package_name>` and `<your_node_name>` with the appropriate package and node names in your ROS 2 workspace.

5. The node will start capturing video from the camera and display it in a window. It will detect and track circles (presumed to be a ball) in the video stream. The coordinates of the detected ball's center (normalized) will be displayed on the video frame.

6. Press the 'q' key to exit the program.

## Parameters

You can customize the behavior of the ball tracking by modifying the parameters in the `tracker` method of the `MyNode` class. Here are some important parameters you can adjust:

- `minDist`: Minimum distance between the centers of detected circles.
- `param1` and `param2`: Parameters for circle detection using the Hough Circle Transform.
- `minRadius` and `maxRadius`: Minimum and maximum radii of the circles to be detected.

