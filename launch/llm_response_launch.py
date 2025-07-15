#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """生成启动描述"""
    
    llm_response_node = Node(
        package='llm_response_package',
        executable='llm_response_node',
        name='llm_response_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('llm_response_package'), 'config', 'llm_config.yaml')
        ],
    )
    
    return LaunchDescription([
        llm_response_node
    ]) 