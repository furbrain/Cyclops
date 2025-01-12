from pathlib import Path
from typing import Set, List
import subprocess

def find_x_cams(x: str) -> Set[int]:
    pth = Path("/sys/devices")
    devs = pth.glob("**/*usb*/**/video?")
    return set(int(str(x)[-1]) for x in devs)

def find_usb_cams() -> Set[int]:
    return find_x_cams("usb")

def find_csi_cams() -> Set[int]:
    return find_x_cams("csi")

def matches_format(dev:int, fmt:bytes) -> bool:
    output: bytes = subprocess.check_output(["v4l2-ctl", "-d", str(dev), "--list-formats"])
    return fmt in output

def get_matching_formats(nums: Set[int], fmt:bytes) -> List[int]:
    return [int(x) for x in nums if matches_format(x, fmt)]


if __name__=="__main__":
    usb_cams = find_usb_cams()
    print(get_matching_formats(usb_cams, b"MJPG"))