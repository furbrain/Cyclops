from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

tuning_file = os.path.join(get_package_share_directory('cyclops'), 'config/imx462_beta.json')
STAGGER_DELAY = 8.0  # seconds to wait before starting second camera
IPA_ENV = {
    'LIBCAMERA_RPI_TUNING_FILE': tuning_file,
    'LIBCAMERA_LOG_LEVELS': '*:WARN,IPARPI:DEBUG',  # logs per-frame AGC exposure/gain
}
PARAMS = {
    'SyncFrames': 10,   # should match between client and server
    'orientation': 180,
    'FrameDurationLimits': [100000, 100000], #10fps
    'width': 640,
    'height': 480,
    'ScalerCrop': [240, 0, 1440, 1080],
    'format': "RGB888"
#    'sensor_mode': '1920:1080',
#    'role': 'video',
}

REMAPPINGS = [('~/'+x,x) for x in ('image_raw', 'image_raw/compressed', 'camera_info', 'set_camera_info')]

# def generate_launch_description():
#     left_camera = Node(
#         package='camera_ros',
#         executable='camera_node',
#         name='camera',
#         namespace='left',
#         parameters=[{
#             'camera': "/base/axi/pcie@1000120000/rp1/i2c@70000/arducam_pivariety@c",
#             'SyncMode': 2,      # SyncModeClient (should be started first)
#             'frame_id': 'left',
#             'camera_info_url': 'file://${ROS_HOME}/camera_info/left.yaml'
#         }, PARAMS],
#         remappings=REMAPPINGS,
#         additional_env=IPA_ENV,
#     )
#
#     right_camera = TimerAction(
#         period=STAGGER_DELAY,
#         actions=[
#             Node(
#                 package='camera_ros',
#                 executable='camera_node',
#                 name='camera',
#                 namespace='right',
#                 parameters=[{
#                     'camera': "/base/axi/pcie@1000120000/rp1/i2c@88000/arducam_pivariety@c",
#                     'SyncMode': 1,      # SyncModeServer
#                     'frame_id': 'right',
#                     'camera_info_url': 'file://${ROS_HOME}/camera_info/right.yaml'
#                 }, PARAMS],
#                 remappings=REMAPPINGS,
#                 additional_env=IPA_ENV,
#             )
#         ]
#     )
#
#     return LaunchDescription([
#         left_camera,
#         right_camera,
#     ])


# Per-camera config: namespace, i2c path, SyncMode (2=client/left, 1=server/right)
CAMERAS = [
    ('left', '/base/axi/pcie@1000120000/rp1/i2c@70000/arducam_pivariety@c', 2),
    ('right', '/base/axi/pcie@1000120000/rp1/i2c@88000/arducam_pivariety@c', 1),
]

# Both cameras MUST run in the same process. The Arducam Pi IPA reads each
# camera's compressed tuning blob from the camera MCU over raw /dev/i2c-N during
# libcamera CameraManager::start(), for *every* camera in the system. Two
# camera_node processes therefore read both MCUs concurrently, interleave their
# I2C transactions, and abort with "Exception during zlib decompression".
# camera_ros shares one CameraManager between components in a process, so the
# IPA initialisation happens once, serially.


def make_camera_component(namespace, camera_path, sync_mode):
    return ComposableNode(
        package='camera_ros',
        plugin='camera::CameraNode',
        name='camera',
        namespace=namespace,
        parameters=[{
            'camera': camera_path,
            'SyncMode': sync_mode,
            'frame_id': namespace,
            'camera_info_url': f'file://${{ROS_HOME}}/camera_info/{namespace}.yaml',
        }, PARAMS],
        remappings=REMAPPINGS,
    )


def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            package='rclcpp_components',
            executable='component_container',
            name='camera_container',
            namespace='',
            composable_node_descriptions=[
                make_camera_component(namespace, camera_path, sync_mode)
                for namespace, camera_path, sync_mode in CAMERAS
            ],
            additional_env=IPA_ENV,
            output='screen',
        )
    ])
