from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # First node
    first_node = Node(
        package='cyclops',
        executable='orb_make_settings',
        name='orb_make_settings',
    )

    after_first_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('cyclops'), 'launch', 'orb_main.yaml'])
        )
    )

    # Start second_node only after first_node finishes
    start_second_after_first = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=first_node,
            on_exit=[after_first_launch]
        )
    )

    return LaunchDescription([
        first_node,
        start_second_after_first
    ])
