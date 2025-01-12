from pathlib import Path
from typing import Set, List
import subprocess

def find_x_cams(x: str) -> Set[int]:
    pth = Path("/sys/devices")
    devs = pth.glob(f"**/*{x}*/**/video?")
    return set(int(str(x)[-1]) for x in devs)

def matches_format(dev:int, fmt:bytes) -> bool:
    output: bytes = subprocess.check_output(["v4l2-ctl", "-d", str(dev), "--list-formats"])
    return fmt in output

def find_cams(cam_type: str, fmt:str = None) -> List[int]:
    cams = find_x_cams(cam_type)
    if fmt is None:
        return list(cams)
    return [int(x) for x in cams if matches_format(x, bytes(fmt,'UTF8'))]


if __name__=="__main__":
    find_cams("usb","H264")