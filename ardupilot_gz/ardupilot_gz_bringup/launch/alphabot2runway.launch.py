# Copyright 2023 ArduPilot.org.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
 
"""Launch an AlphaBot2 differential drive robot in Gazebo and RViz."""
import os
from pathlib import Path
 
from ament_index_python.packages import get_package_share_directory
 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
 
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
 
def generate_launch_description():
    """Generate a launch description for the AlphaBot2."""
    # Same packages as the working iris/atlas launches
    pkg_ardupilot_gazebo = get_package_share_directory("ardupilot_gazebo")
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
 
    # AlphaBot2 robot (bridge + state publisher).
    alphabot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_gz_bringup"),
                        "launch",
                        "robots",
                        "alphabot2.launch.py",
                    ]
                ),
            ]
        )
    )
 
    # Gazebo server — world file lives in ardupilot_gazebo, same as other models.
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={
            "gz_args": "-v4 -s -r "
            + os.path.join(pkg_ardupilot_gazebo, "worlds", "alphabot2_runway.sdf")
        }.items(),
    )
 
    # Gazebo GUI.
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={"gz_args": "-v4 -g"}.items(),
    )
 
    # RViz (optional).
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_project_bringup, "rviz", "alphabot2.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
 
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            gz_sim_server,
            gz_sim_gui,
            alphabot2,
            rviz,
        ]
    )