import os

from launch import LaunchIntrospector
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.actions import LogInfo
from launch.actions import EmitEvent

from launch.events import matches_action

from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

from ament_index_python.packages import get_package_share_directory

import lifecycle_msgs.msg


# from launch_ros.substitutions import FindPackageShare
# from launch import LaunchDescription
# from launch_ros.actions import Node


# import launch


# from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
#                             LogInfo, RegisterEventHandler, TimerAction)
# from launch.conditions import IfCondition
# from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
#                                 OnProcessIO, OnProcessStart, OnShutdown)
# from launch.events import Shutdown
# from launch.substitutions import (EnvironmentVariable, FindExecutable,
#                                 LaunchConfiguration, LocalSubstitution,
#                                 PythonExpression)


# from launch_ros.events.lifecycle import ChangeState
# from launch.events import matches_action

# import launch_ros.actions  # noqa: E402
# import launch_ros.events  #n oqa: E402
# import launch_ros.events.lifecycle  # noqa: E402

# from launch_ros.actions import LifecycleNode

def generate_launch_description():

    ns = LaunchConfiguration('ns')

    ns_launch_arg = DeclareLaunchArgument(
        'ns',
        default_value='',
        description='Namespace of the nodes'
        )

    config_file_path = os.path.join(
        get_package_share_directory('actuator_bringup'),
        'config',
        'rmd_config.yaml'
        )

    actuator_driver_node = LifecycleNode(
        package='actuator_driver',
        namespace=ns,
        executable='actuator_driver',
        name='actuator_driver',
        parameters=[config_file_path],
        prefix=['stdbuf -o L'],
        arguments=['--ros-args', '--log-level', "actuator_driver:=debug"],
    )

    # ros2 lifecycle set /rmd_actuator/actuator_driver configure
    configure_actuator_driver = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' lifecycle set ',
            ns,
            '/actuator_driver ',
            'configure'
        ]],
        shell=True
    )

    activate_actuator_driver = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' lifecycle set ',
            ns,
            '/actuator_driver ',
            'activate'
        ]],
        shell=True
    )

    shutdown_actuator_driver = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' lifecycle set ',
            ns,
            '/actuator_driver ',
            'shutdown'
        ]],
        shell=True
    )


    # When the node reaches the 'inactive' state, make it take the 'activate' transition.
    event_actuator_driver_reaches_inactive_state = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=actuator_driver_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="> actuator_driver_node reached the 'inactive' state, 'activating'."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(actuator_driver_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    ### Start building the launch description ###
    ld = LaunchDescription()

    ld.add_action(ns_launch_arg)
    ld.add_action(actuator_driver_node)

    ld.add_action(event_actuator_driver_reaches_inactive_state)
    ld.add_action(configure_actuator_driver)

    print('Starting introspection of launch description...')
    print('-----------------------------------------------')
    print(LaunchIntrospector().format_launch_description(ld))
    print('-----------------------------------------------')

    return ld