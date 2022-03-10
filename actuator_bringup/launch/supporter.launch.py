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

    config_file_path = os.path.join(
        get_package_share_directory('actuator_bringup'),
        'config',
        'rmd_config.yaml'
        )

    # Include base file (GroupAction prevent ns of this file to be overwritten)
    include_rmd_base_launch = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rmd_actuator_base.launch.py']),
            launch_arguments={'ns': ns}.items(),
            )]
    )

    supporter_controller_node = LifecycleNode(
        package='supporter_controller',
        namespace=ns,
        executable='supporter_controller',
        name='supporter_controller',
        parameters=[config_file_path],
        prefix=['stdbuf -o L'],
        arguments=['--ros-args', '--log-level', "supporter_controller:=debug"],
    )

    # Make the talker node take the 'configure' transition.
    emit_event_to_request_supporter_controller_does_configure_transition = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(supporter_controller_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )


    # When the node reaches the 'inactive' state, make it take the 'activate' transition.
    event_supporter_controller_reaches_inactive_state = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=supporter_controller_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="> supporter_controller_node reached the 'inactive' state, 'activating'."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(supporter_controller_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )


    ### Start building the launch description ###
    ld = LaunchDescription()

    ld.add_action(ns_launch_arg)
    ld.add_action(include_rmd_base_launch)
    ld.add_action(supporter_controller_node)

    ld.add_action(event_supporter_controller_reaches_inactive_state)
    ld.add_action(emit_event_to_request_supporter_controller_does_configure_transition)

    print('-----------------------------------------------')
    print(' => ' + os.path.basename(__file__))
    print('-----------------------------------------------')
    print(LaunchIntrospector().format_launch_description(ld))
    print('-----------------------------------------------')

    return ld