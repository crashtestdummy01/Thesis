"""
Eye-on-base hand-eye calibration for ONE camera at a time.

Usage (robot bringup, RealSense driver and aruco_tf_detector must already be running):
  ros2 launch /path/to/handeye_calibrate.launch.py cam:=cam1
  ros2 launch /path/to/handeye_calibrate.launch.py cam:=cam2

Naming convention:
  calibration name       : <cam>_eob      (e.g. cam1_eob)
  tracking_base_frame    : <cam>_link     (root of the RealSense tf tree)
  tracking_marker_frame  : <cam>_marker   (the marker_frame param of your detector)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    cam = LaunchConfiguration("cam")

    calibrate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("easy_handeye2"),
                "launch",
                "calibrate.launch.py",
            )
        ),
        launch_arguments={
            "calibration_type": "eye_on_base",
            "name": [cam, "_eob"],
            # robot side (tf frames, no leading slash in ROS 2)
            "robot_base_frame": LaunchConfiguration("robot_base_frame"),
            "robot_effector_frame": LaunchConfiguration("robot_effector_frame"),
            # camera side
            "tracking_base_frame": [cam, "_link"],
            "tracking_marker_frame": [cam, "_marker"],
            # Humble: no automatic MoveIt movement, you move the arm yourself
            "freehand_robot_movement": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("cam", default_value="cam1"),
            DeclareLaunchArgument("robot_base_frame", default_value="fr3_link0"),
            # the link the board is rigidly mounted to
            DeclareLaunchArgument("robot_effector_frame", default_value="fr3_link8"),
            calibrate,
        ]
    )
