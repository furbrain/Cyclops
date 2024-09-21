from reader.gstreader import VidReader, TOFReader, IMUReader

filename = "/home/phil/tmp2.mkv"

READERS = {
    VidReader: f"""filesrc location="{filename}" ! matroskademux ! video/x-h264 ! decodebin ! 
                            videoconvert ! video/x-raw,format=GRAY8 ! appsink sync=false""",
    TOFReader: f'''filesrc location="{filename}" ! matroskademux !
          video/x-raw,width=480,format=GRAY8 ! appsink sync=false''',
    IMUReader: f"""filesrc location="{filename}" name=fsrc ! matroskademux name=demux !  
                            video/x-raw ! appsink name=telemetry sync=false"""
}

for cls, spec in READERS.items():
    last_tm = 0
    with cls(spec) as reader:
        print(f"Class: {cls.__name__}")
        while True:
            tm, frame = reader.get_frame()
            if frame is None:
                break
            if tm - last_tm > 0.1:
                print(f"    Glitch at: {tm}; {tm - last_tm}")
            last_tm = tm