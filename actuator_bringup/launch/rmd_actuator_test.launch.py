import os

from launch import LaunchIntrospector
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.actions import LogInfo
from launch.actions import EmitEvent
from launch.actions import GroupAction
from launch_ros.actions import Node

from launch.events import matches_action

from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

from ament_index_python.packages import get_package_share_directory

import lifecycle_msgs.msg


from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():

    ns = LaunchConfiguration('ns')

    ns_launch_arg = DeclareLaunchArgument(
        'ns',
        default_value='',
        description='Prefix for node names'
        )

    actuator_action_node = Node(
        package='actuator_action',
        namespace=ns,
        executable='actuator_action',
        name='actuator_action',
        # prefix=['stdbuf -o L'],
        arguments=['--ros-args', '--log-level', "actuator_action:=debug"],
    )

    # Include base file (GroupAction prevent ns of this file to be overwritten)
    include_rmd_base_launch = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rmd_actuator.launch.py']),
        launch_arguments={'ns': ns}.items(),
    )])

    ### Start building the launch description ###
    ld = LaunchDescription()

    ld.add_action(ns_launch_arg)
    ld.add_action(include_rmd_base_launch)
    ld.add_action(actuator_action_node)


    print('Starting introspection of launch description...')
    print('-----------------------------------------------')
    print(LaunchIntrospector().format_launch_description(ld))
    print('-----------------------------------------------')

    return ld