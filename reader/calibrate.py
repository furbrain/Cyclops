import csv
import struct
import sys
import time

import gi
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject, GLib

gi.require_version('GstVideo', '1.0')
from gi.repository import GstVideo
from imu import IMUTracker

spec_simple = """filesrc location="combined.mkv" name=fsrc ! matroskademux name=demux !  
                 video/x-raw ! appsink name=telemetry sync=false"""


def on_message(bus, message):
    print("Message: ", message)




def on_sample(appsink, csv_file):
    global count
    sample = appsink.emit("pull-sample")
    if sample is not None:
        buffer = sample.get_buffer()
        result, mapinfo = buffer.map(Gst.MapFlags.READ)
        try:
            floats = struct.unpack('<9d', mapinfo.data[:72])
        finally:
            buffer.unmap(mapinfo)
        csv_file.writerow((buffer.pts/1_000_000_000, *floats))
    return Gst.FlowReturn.OK


def main():
    Gst.init(sys.argv)

    pipeline = Gst.parse_launch(spec_simple)
    pipeline.set_state(Gst.State.PLAYING)
    telemetry = pipeline.get_by_name("telemetry")

    with open("telemetry.csv","w") as f:
        csv_file = csv.writer(f)
        csv_file.writerow(("PTS", "Mx", "My", "Mz", "Ax", "Ay", "Az", "Gx", "Gy", "Gz"))

        bus = pipeline.get_bus()
        bus.connect("message", on_message)
        telemetry.set_property("emit-signals", True)
        telemetry.connect("new-sample", on_sample, csv_file)
        while True:
            time.sleep(0.1)
            message = bus.pop_filtered(Gst.MessageType.EOS)
            if message:
                print(message.type)
                break
    print("complete, shutting down")
    pipeline.set_state(Gst.State.NULL)
    time.sleep(0.5)

main()
