from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.events import matches_action
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

import lifecycle_msgs.msg
import os

def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'parameters_file',
            default_value=PathJoinSubstitution([FindPackageShare('laser_trajetory_points'),
                                                'params', 'waypoint.yaml']),
            description=''
        )
    )

    parameters_file = LaunchConfiguration('parameters_file')

    calculator_lifecycle_node = LifecycleNode(
    package='laser_trajetory_points', 
    executable='laser_trajetory_points',
    name='laser_trajetory_points',
    namespace='',
    output='screen',
    parameters=[parameters_file],
    remappings=[]
)

    event_handlers = []

    event_handlers.append(
        RegisterEventHandler(
            OnProcessStart(
                target_action=calculator_lifecycle_node,
                on_start=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(calculator_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                ],
            )
        ),
    )

    event_handlers.append(
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=calculator_lifecycle_node,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(calculator_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        ),
    )

    ld = LaunchDescription()

    for argument in declared_arguments:
        ld.add_action(argument)

    ld.add_action(calculator_lifecycle_node)

    for event_handler in event_handlers:
        ld.add_action(event_handler)

    return ld
