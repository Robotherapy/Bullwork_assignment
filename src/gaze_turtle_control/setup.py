from setuptools import find_packages, setup

package_name = 'gaze_turtle_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/gaze_turtle_control/launch', ['launch/gaze_turtle_launch.py']),
    ],
    install_requires=['setuptools', 'opencv-python', 'rclpy'],
    zip_safe=True,
    maintainer='robotherapy',
    maintainer_email='robotherapyy@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'webcam_reader_node = gaze_turtle_control.webcam_reader_node:main',
        'gaze_tracker_node = gaze_turtle_control.gaze_tracker_node:main',
        'turtle_controller_node = gaze_turtle_control.turtle_controller_node:main',
        'camera_viewer_node = gaze_turtle_control.camera_viewer_node:main',
    ],
    },
)
