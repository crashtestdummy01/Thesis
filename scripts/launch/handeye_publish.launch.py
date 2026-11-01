"""
Publishes the saved hand-eye calibrations of both cameras to tf:
  panda_link0 -> cam1_link   and   panda_link0 -> cam2_link

Run it alongside your robot bringup and camera drivers, or include it from your
own bringup launch file.
  ros2 launch /path/to/handeye_publish.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

CALIBRATION_NAMES = ["cam1_eob", "cam2_eob"]  # must match the names used when calibrating


def generate_launch_description():
    publish_file = os.path.join(
        get_package_share_directory("easy_handeye2"), "launch", "publish.launch.py"
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(publish_file),
                launch_arguments={"name": name}.items(),
            )
            for name in CALIBRATION_NAMES
        ]
    )
