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

    actuator_rpi_node = LifecycleNode(
        package='actuator_rpi',
        namespace=ns,
        executable='actuator_rpi',
        name='actuator_rpi',
        parameters=[config_file_path],
        prefix=['stdbuf -o L'],
        arguments=['--ros-args', '--log-level', "actuator_rpi:=debug"],
    )

    # Make the talker node take the 'configure' transition.
    emit_event_to_request_actuator_driver_does_configure_transition = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(actuator_driver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the talker node take the 'configure' transition.
    emit_event_to_request_actuator_rpi_does_configure_transition = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(actuator_rpi_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
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
                emit_event_to_request_actuator_rpi_does_configure_transition,
            ],
        )
    )

    # When the node reaches the 'inactive' state, make it take the 'activate' transition.
    event_actuator_rpi_reaches_inactive_state = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=actuator_rpi_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="> actuator_rpi_node reached the 'inactive' state, 'activating'."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(actuator_rpi_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
                # emit_event_to_request_supporter_controller_does_configure_transition,
            ],
        )
    )

    ### Start building the launch description ###
    ld = LaunchDescription()

    ld.add_action(ns_launch_arg)
    ld.add_action(actuator_driver_node)
    ld.add_action(actuator_rpi_node)

    ld.add_action(event_actuator_driver_reaches_inactive_state)   
    ld.add_action(event_actuator_rpi_reaches_inactive_state)
    ld.add_action(emit_event_to_request_actuator_driver_does_configure_transition)

    print('-----------------------------------------------')
    print(' => ' + os.path.basename(__file__))
    print('-----------------------------------------------')
    print(LaunchIntrospector().format_launch_description(ld))
    print('-----------------------------------------------')

    return ld