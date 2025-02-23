# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2025 Phil Underwood for Underwood Underground
#
# SPDX-License-Identifier: Unlicense
import sys
sys.path.insert(0,'..')
import board
from cyclops_imu import icm
import time
import digitalio
import busio
from cyclops_imu.ros_adapter import ROS_Adapter
import rospy


en_pin = digitalio.DigitalInOut(board.D4)
en_pin.switch_to_output(False)
i2c_bus = busio.I2C(board.SCL, board.SDA, frequency=400_000)
device = icm.ICM20948(i2c_bus)
ros = ROS_Adapter.create_from_sensor(device, 
                                     frame_id=rospy.get_param("~frame_id","imu"),
                                     rate=rospy.get_param("~rate",100.0))
ros.run()

