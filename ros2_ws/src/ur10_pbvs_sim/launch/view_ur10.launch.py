from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ur10_pbvs_sim"),
                                "launch",
                                "view_ur10.launch.xml",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "ur_type": LaunchConfiguration("ur_type", default="ur10"),
                }.items(),
            )
        ]
    )
