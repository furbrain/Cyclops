from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

tuning_file = os.path.join(get_package_share_directory('cyclops'), 'config/imx462.json')
STAGGER_DELAY = 5.0  # seconds to wait before starting second camera

#IPA_ENV = {'LIBCAMERA_IPA_FORCE_ISOLATION': '1'}
IPA_ENV = {'LIBCAMERA_RPI_TUNING_FILE': tuning_file}
PARAMS = {
    'SyncFrames': 10,   # should match between client and server
    'orientation': 180,
    'FrameDurationLimits': [100000, 100000],
#    'width': 640,
#    'height': 480,
#    'role': 'video',
}

REMAPPINGS = [('~/'+x,x) for x in ('image_raw', 'image_raw/compressed', 'camera_info', 'set_camera_info')]

def generate_launch_description():
    left_camera = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera',
        namespace='left',
        parameters=[{
            'camera': "/base/axi/pcie@1000120000/rp1/i2c@88000/arducam_pivariety@c",  # adjust to match your device index or path
            'SyncMode': 2,      # SyncModeClient (should be started first)
            'frame_id': 'left',
        }, PARAMS],
        remappings=REMAPPINGS,
        additional_env=IPA_ENV,
    )

    right_camera = TimerAction(
        period=STAGGER_DELAY,
        actions=[
            Node(
                package='camera_ros',
                executable='camera_node',
                name='camera',
                namespace='right',
                parameters=[{
                    'camera': "/base/axi/pcie@1000120000/rp1/i2c@70000/arducam_pivariety@c",  # adjust to match your device index or path
                    'SyncMode': 1,      # SyncModeServer
                    'frame_id': 'right',
                }, PARAMS],
                remappings=REMAPPINGS,
                additional_env=IPA_ENV,
            )
        ]
    )

    return LaunchDescription([
        left_camera,
        right_camera,
    ])
