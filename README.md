Bullwork_assignment
Gaze Controlled TurtleSim

This package allows controlling the Turtlesim robot using gaze direction, simulated using basic face detection. It uses ROS 2 and OpenCV to process webcam input, detect the face, estimate gaze direction, and send appropriate velocity commands to Turtlesim.

Overview

This package consists of four ROS 2 nodes:

    webcam_reader_node: Captures frames from the webcam and publishes them as sensor_msgs/Image.

    gaze_tracker_node: Processes the image, detects face position, and publishes a normalized gaze vector as geometry_msgs/Vector3.

    turtle_controller_node: Reads the gaze vector and sends corresponding geometry_msgs/Twist commands to the /turtle1/cmd_vel topic.

    camera_viewer_node: Subscribes to the webcam image and overlays visual face detection (for debugging and verification).

The system is designed to run using a single launch file that starts all the components including Turtlesim.

Dependencies

    ROS 2 Humble

    OpenCV (via Python)

    cv_bridge

    sensor_msgs, geometry_msgs

    turtlesim

Installation

    Clone the repository into a ROS 2 workspace:
    cd ~/bullwork_ws/src
    git clone https://github.com/Robotherapy/Bullwork_assignment.git

    Build the workspace:
    cd ~/bullwork_ws
    colcon build
    source install/setup.bash

    Ensure the launch file has executable permission:
    chmod +x src/gaze_turtle_control/launch/gaze_turtle_launch.py

Running the System

Use the following command to launch the system:
ros2 launch gaze_turtle_control gaze_turtle_launch.py

To run the debug viewer with face bounding boxes:
ros2 run gaze_turtle_control camera_viewer_node

Configuration

The file config/params.yaml contains parameters that control sensitivity and motion behavior. You can modify them as needed:

webcam_reader_node:
ros__parameters:
frame_rate: 30.0

gaze_tracker_node:
ros__parameters:
gaze_sensitivity: 1.2

turtle_controller_node:
ros__parameters:
max_linear_speed: 1.5
max_angular_speed: 2.0
dead_zone: 0.1

Descriptions:

    frame_rate: Webcam capture rate.

    gaze_sensitivity: Multiplier for detected gaze offset.

    max_linear_speed: Forward/backward speed for the turtle.

    max_angular_speed: Turning speed.

    dead_zone: Tolerance near the center where no movement is sent.

Gaze to Motion Mapping

The gaze vector is determined by the detected face position relative to the image center:

    Looking left/right turns the turtle.

    Looking up/down moves it forward or backward.

    No significant movement is interpreted as "stay".

Gaze values are normalized between -1.0 and 1.0. Movement is ignored if within the dead_zone.

File Structure

gaze_turtle_control/

├── config/

│ └── params.yaml

├── launch/

│ └── gaze_turtle_launch.py

├── gaze_turtle_control/

│ ├── webcam_reader_node.py

│ ├── gaze_tracker_node.py

│ ├── turtle_controller_node.py

│ └── camera_viewer_node.py

├── package.xml

└── setup.py

Known Issues

    Turtlesim may move in circles if your face is off-center but not moving.

    Face detection is basic; no real gaze estimation.

    Works best when face is fully visible and evenly lit.

Extending

You can extend this system by:

    Replacing the face detection logic with actual gaze estimation (e.g., using deep learning).

    Adding smoothing filters to the gaze vector to avoid jitter.

    Mapping gaze vectors to more complex motion profiles.

License

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0  

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

I used ChatGpt to generate this, but I could only get it working because I understand how to manage ROS2 projects and work on them.
