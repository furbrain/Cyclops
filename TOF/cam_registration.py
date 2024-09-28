import cv2
import numpy as np

from reader.LensCalibration import Lens
from reader.gstreader import VidReader, TOFReader

FILENAME = "/home/phil/cal.mkv"

vid = VidReader.from_filename(FILENAME, gray=False, sync=False)
tof = TOFReader.from_filename(FILENAME, absolute=True)
lens = Lens.load("../reader/lens_params.npz")

CHECKERBOARD = (6,9)
homography = None
tm_t = 0
with vid, tof:
    for tm_v, col_v in vid:
        #col_v = lens.undistort([col_v])[0]
        gray_v = cv2.cvtColor(col_v, cv2.COLOR_BGR2GRAY)
        while tm_t < tm_v:
            tm_t, frame_t = tof.get_frame()
            if tm_t is None:
                break
        if tm_t is None:
            break
        frame_t = cv2.normalize(frame_t, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        frame_t = cv2.rotate(frame_t, cv2.ROTATE_90_COUNTERCLOCKWISE)
        if homography is None:
            ret_v, corners_v = cv2.findChessboardCorners(gray_v, CHECKERBOARD,
                                                         cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
            ret_t, corners_t = cv2.findChessboardCorners(frame_t, CHECKERBOARD,
                                                 cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if homography is None:
            cv2.imshow("Vid", col_v)
            cv2.imshow("Tof", frame_t)
            key = cv2.waitKey(33)
            if key == ord(' '):
                homography, _ = cv2.findHomography(corners_v, corners_t)
                vid_warp = cv2.warpPerspective(col_v, homography, (180, 240))
                cv2.imshow("warp", vid_warp)
                cv2.imshow("Vid", col_v)
                cv2.imshow("Tof", frame_t)
                cv2.waitKey(0)
        else:
            vid_warp = cv2.warpPerspective(col_v,homography,(180,240))
            cv2.imshow("warp", vid_warp)
            cv2.imshow("Vid", col_v)
            cv2.imshow("Tof", frame_t)
            key = cv2.waitKey(33)
            if key == ord('q'):
                break
            if key == ord(' '):
                cv2.waitKey(0)
if homography is not None:
    np.save("homography.npy", homography)