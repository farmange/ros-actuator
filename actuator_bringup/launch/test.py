import os
from launch import LaunchIntrospector
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

def generate_launch_description():

    ld = LaunchDescription()

    node = Node(
        package='actuator_driver',
        namespace='/',
        executable='actuator_driver',
        name='node',
        arguments=['--ros-args', '--log-level', "INFO"],
        # on_shutdown=launch.actions.Shutdown()
    )
    lc_node = LifecycleNode(
        package='actuator_driver',
        namespace='/',
        executable='actuator_driver',
        name='lc_node',
        arguments=['--ros-args', '--log-level', "INFO"],
        # on_shutdown=launch.actions.Shutdown()
    )

    ld.add_action(node)
    # ld.add_action(lc_node)

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    return ld
