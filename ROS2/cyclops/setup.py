import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cyclops'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		    "svc = cyclops.simple_service:main",
            "calibrate = cyclops.calibrate:main",
            "calibrate_dual = cyclops.calibrate_dual:main",
            "calibrate_cam_imu = cyclops.calibrate_cam_imu:main",
            "cameras = cyclops.cameras:main",
            "tof = cyclops.tof:main",
            "rgb = cyclops.rgb:main",
            "tf2_dynamic = cyclops.tf2_dynamic:main",
            "orb_make_settings = cyclops.orb_make_settings:main",
            "register = cyclops.register:main",
            "beeper = cyclops.beeper:main",
            "switcher = cyclops.switcher:main",
            "tracker  = cyclops.tracker:main",
            "orb_beeper = cyclops.orb_beeper:main",
            "orb_state_beeper = cyclops.orb_state_beeper:main",
            "modeller = cyclops.modeller:main"
        ],
    },
)
