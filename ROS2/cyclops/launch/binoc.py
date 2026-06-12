from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

tuning_file = os.path.join(get_package_share_directory('cyclops'), 'config/imx462.json')
STAGGER_DELAY = 8.0  # seconds to wait before starting second camera
RESPAWN_DELAY = 2.0 
MAX_RESTARTS=5
IPA_ENV = {'LIBCAMERA_RPI_TUNING_FILE': tuning_file}
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


def make_camera_node(namespace, camera_path, sync_mode):
    return Node(
        package='camera_ros',
        executable='camera_node',
        name='camera',
        namespace=namespace,
        parameters=[{
            'camera': camera_path,
            'SyncMode': sync_mode,
            'frame_id': namespace,
            'camera_info_url': f'file://${{ROS_HOME}}/camera_info/{namespace}.yaml',
        }, PARAMS],
        remappings=REMAPPINGS,
        additional_env=IPA_ENV,
        respawn=True,
        respawn_max_retries=MAX_RESTARTS,
        respawn_delay=RESPAWN_DELAY
    )


def generate_launch_description():
    return LaunchDescription([
        make_camera_node(namespace, camera_path, sync_mode)
        for namespace, camera_path, sync_mode in CAMERAS
    ])
