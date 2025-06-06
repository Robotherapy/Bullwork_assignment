from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node',
            output='screen'
        ),
        Node(
            package='gaze_turtle_control',
            executable='webcam_reader_node',
            name='webcam_reader_node',
            parameters=['config/params.yaml']
        ),
        Node(
            package='gaze_turtle_control',
            executable='gaze_tracker_node',
            name='gaze_tracker_node',
            parameters=['config/params.yaml']
        ),
        Node(
            package='gaze_turtle_control',
            executable='turtle_controller_node',
            name='turtle_controller_node',
            parameters=['config/params.yaml']
        ),
        Node(
    	    package='gaze_turtle_control',
    	    executable='camera_viewer_node',
    	    name='camera_viewer_node',
    	    output='screen',
    	    parameters=['config/params.yaml']
	)

    ])
