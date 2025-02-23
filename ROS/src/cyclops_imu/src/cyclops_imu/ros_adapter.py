# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2025 Phil Underwood for Underwood Underground
#
# SPDX-License-Identifier: MIT
"""
`ros_adapter`
================================================================================

A library to simplify using Circuitpython style sensors with ROS (Robot Operating System)


* Author(s): Phil Underwood

Implementation Notes
--------------------


**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Robot Operating System: https://ros.org/
"""
try:
    from typing import Dict, Type, List
except ImportError:
    pass

from struct import Struct
# imports
try:
    import rospy
    from rospy import Publisher, Duration
    import genpy
    ROS_VERSION=1
except ImportError:
    import rclpy
    from rclpy.publisher import Publisher
    import genpy
    ROS_VERSION=2

class Converter:
    def __init__(self, cls, field_name, source_name=None, scale=1.0):
        self.cls = cls
        if source_name is None:
            self.source_names = field_name
        else:
            self.source_names = [source_name]
        self.field_name = field_name
        self.scale = scale

    def assign(self, source) -> genpy.Message:
        msg = self.cls()
        value = getattr(source, self.source_names[0]) * self.scale
        msg.setattr(self.field_name, value)
        return msg


if ROS_VERSION in (1,2):
    # same basic message types in ROS1 and ROS2
    from std_msgs.msg import ColorRGBA
    from sensor_msgs.msg import (FluidPressure, Illuminance, Imu, MagneticField, RelativeHumidity,
                                 Temperature, Range)

    class ColorConverter(Converter):
        _STRCT = Struct(">I")

        def __init__(self):
            super().__init__(ColorRGBA, "color")

        def assign(self, source):
            value = getattr(source, "color")
            obj = self.cls()
            chars = [x / 255 for x in self._STRCT.pack(value)]
            obj.r, obj.g, obj.b, obj.a = chars
            return obj

    class IMUConverter(Converter):
        # FIXME: quaternion/orientation support
        def __init__(self):
            super().__init__(Imu,None)
            self.source_names = ["gyro", "acceleration"]

        def assign(self, source):
            obj = self.cls()
            if hasattr(source, "gyro"):
                g = obj.angular_velocity
                g.x, g.y, g.z = source.gyro
            if hasattr(source, "acceleration"):
                a = obj.linear_acceleration
                a.x, a.y, a.z = source.acceleration
            return obj

    class MagneticConverter(Converter):
        def __init__(self):
            super().__init__(MagneticField, "magnetic_field", "magnetic", scale=1 / 1_000_000)
            
        def assign(self, source):
            obj = self.cls()
            m = obj.magnetic_field
            m.x, m.y, m.z = [x*self.scale for x in source.magnetic]
            return obj
    

    SENSOR_TYPES: List[Converter] = [
        Converter(FluidPressure, "fluid_pressure", "pressure", scale=0.01), # hectoPascals: Pascals
        Converter(Illuminance, "lux"), # lux: lux
        IMUConverter(), # rad/s : rad/s
        MagneticConverter(),
        Converter(RelativeHumidity, "relative_humidity", scale=0.01), # percent: 0.0 .. 1.0
        Converter(Temperature, "temperature"), # celsius: celsius
        Converter(Range, "range", "distance", 0.01), # cm: m
        ColorConverter(), # RGB 8bit
    ]

class ROS_Adapter:
    def __init__(self, device, rate:float=100.0, name: str = None, frame_id: str = None):
        self.name = name
        self.rate = rate
        self.device = device
        self.frame_id = frame_id
        self.msgs: Dict[Converter, Publisher] = {}
        for cvt in SENSOR_TYPES:
            if any(hasattr(device, attr) for attr in cvt.source_names):
                self.msgs[cvt] = self._create_publisher(cvt)

    def _create_publisher(self, cvt:Converter):
        # this should be overridden
        raise NotImplementedError

    def _publish_messages(self, event=None):
        for cvt, pub in self.msgs.items():
            msg = cvt.assign(self.device)
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            pub.publish(msg)

    def run(self):
        # this should be overridden in base class
        raise NotImplementedError

    @classmethod
    def create_from_sensor(cls, object, rate: float=0.1, name: str = None, frame_id:str = "base_link"):
        if ROS_VERSION==1:
            return ROS1_Adapter(object, rate, name, frame_id)
        elif ROS_VERSION==2:
            return ROS2_Adapter(object, rate, name, frame_id)

class ROS1_Adapter(ROS_Adapter):

    def __init__(self, object, rate: float = 100.0, name: str = None, frame_id:str = "base_link"):
        if name is None:
            name = object.__class__.__name__
        rospy.init_node(name)
        super().__init__(object, rate, name, frame_id)
        period = Duration(1/float(rate))
        self.timer = rospy.timer.Timer(period, self._publish_messages)

    def _create_publisher(self, cvt: Converter):
        return Publisher(cvt.cls.__name__, cvt.cls, queue_size=1)

    def run(self):
        rospy.spin()

class ROS2_Adapter(ROS_Adapter):
    def __init__(self, object, rate: float = 0.1, name: str = None, frame_id:str = "base_link"):
        if name is None:
            name = object.__class__.__name__
        self.node = rclpy.create_node(name)
        super().__init__(object, rate, name, frame_id)
        self.timer = self.node.create_timer(rate,self._publish_messages)

    def _create_publisher(self, cvt: Converter):
        return self.node.create_publisher(cvt.cls, cvt.cls.__name__)

    def run(self):
        rclpy.spin()


__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/furbrain/CircuitPython_ROS_Adapter.git"
