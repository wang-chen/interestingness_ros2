from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    rviz_config_dir = os.path.join(get_package_share_directory('interestingness_ros2'), 'intere.rviz')
    assert os.path.exists(rviz_config_dir)
    interestingness_node = Node(
        package="interestingness_ros2",
        executable="live_detector",
        parameters=[
            {"image-topic": ["/rs_front/color/image"]},
            {"model-save": '/home/li/CMU_RISS/my_ws2/src/interestingness_ros2/saves/vgg16.pt.SubTF.n100usage.mse'},
            {"crop-size": 320},
            {"num-interest": 10},
            {"skip-frames": 1},
            {"window-size": 1},
            {"save-flag": "test"},
            {"rr": 3250},
            {"wr": 5},
        ]
    )
    rviz_node=Node(package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir]
        )
    marker_node=Node(package='interestingness_ros2',
            executable='interest_marker',
            name='interest_marker',
            parameters=[
            {"min-level": 0.2},
        ]
        )
    ld.add_action(interestingness_node)
    ld.add_action(rviz_node)
    ld.add_action(marker_node)
    return ld