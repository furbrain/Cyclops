import cv2

from reader.LensCalibration import Lens
from reader.gstreader import VidReader

SPEC = """v4l2src device=/dev/video4 ! video/x-h264,width=1280 ! decodebin ! videoconvert ! video/x-raw,
format=GRAY8 ! appsink sync=true"""
cap = VidReader(SPEC)
CHECKERBOARD=(9,6)
frames = []
corners_list = []
with cap:
    for tm, frame in cap:
        col = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        for corners in corners_list:
            col = cv2.drawChessboardCorners(col,CHECKERBOARD,corners, True)
        cv2.imshow("main", col)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        if key == ord(' '):
            print("checking frame")
            ret, corners = cv2.findChessboardCorners(frame, CHECKERBOARD,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
            if ret:
                print("checkerboard found")
                frames.append(frame)
                corners_list.append(corners)
            else:
                print("checkerboard not found")
print(len(frames))
l = Lens()
l.calibrate_from_images(frames, show_results=True)
l.save("lens_params.npz")