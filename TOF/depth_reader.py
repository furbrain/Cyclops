import open3d as o3d
import cv2
import numpy as np
import math as maths
from reader.gstreader import VidReader, TOFReader


FOV_X = 58.5
FOV_Y = 45.6
MAX_WIDTH=240
MAX_HEIGHT=180
#fx = MAX_WIDTH / (2 * maths.tan(0.5 * maths.pi * FOV_X / 180)) # WIDTH / 2 / maths.tan(0.5 * FOV_X)
#fy = MAX_HEIGHT / (2 * maths.tan(0.5 * maths.pi * FOV_Y / 180)) # HEIGHT / 2 / maths.tan(0.5 * FOV_Y)
#fx = 250 # calculated by direct observation
fx = 210
fy = fx



cx = MAX_WIDTH / 2
cy = MAX_HEIGHT / 2
print(fx, fy)

#note x,y numbers are swapped as we are using in portrait mode
TOF_INTRINSIC= o3d.camera.PinholeCameraIntrinsic(MAX_HEIGHT,MAX_WIDTH,fy,fx,cy,cx)


class DepthReader:
    def __init__(self, fname: str, homography: np.ndarray, sync: bool=False):
        self.vid = VidReader.from_filename(fname,sync=sync)
        self.tof = TOFReader.from_filename(fname,sync=sync, absolute=False)
        self.homography = homography
        self.tm_t = 0
        self.tm_v_last = 0
        self.tm_v =0
        self.frame_v_last = None
        self.frame_v = None

    def __enter__(self):
        self.vid.start()
        self.tof.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.vid.close()
        self.tof.close()

    def __iter__(self):
        while True:
            res = self.get_frames()
            if res[0] is None:
                break
            yield res

    def warp_frame(self, frame):
        return cv2.warpPerspective(frame, self.homography,(180,240))

    def make_depth_image(self, frame_t, frame_v):
        if frame_v is None:
            return None
        frame_t[frame_t==0] = np.nan
        frame_t = o3d.geometry.Image(frame_t.astype("float32"))
        frame_v = o3d.geometry.Image(cv2.cvtColor(frame_v, cv2.COLOR_BGR2RGB))
        return o3d.geometry.RGBDImage.create_from_color_and_depth(frame_v, frame_t,
                                                                  depth_scale=1000,
                                                                  depth_trunc=3.5,
                                                                  convert_rgb_to_intensity=False)

    def get_frames(self):
        tm, frame_t = self.tof.get_frame()
        if tm is None:
            return None, None
        frame_t = cv2.rotate(frame_t, cv2.ROTATE_90_COUNTERCLOCKWISE)
        while self.tm_v is not None and self.tm_v < tm:
            self.tm_v_last = self.tm_v
            self.frame_v_last = self.frame_v
            self.tm_v, frame_v = self.vid.get_frame()
            if self.tm_v is not None:
                self.frame_v = self.warp_frame(frame_v)
        if self.tm_v is None:
            return None, None
        elif abs(self.tm_v-tm) > abs(self.tm_v_last-tm):
            return tm, self.make_depth_image(frame_t, self.frame_v_last)
        else:
            return tm, self.make_depth_image(frame_t, self.frame_v)

