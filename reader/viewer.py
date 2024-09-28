import cv2
import numpy as np

from reader.gstreader import VidReader, TOFReader

FILENAME = "/home/phil/comb2.mkv"


vid = VidReader.from_filename(FILENAME, sync=False)
tof = TOFReader.from_filename(FILENAME, sync=False, absolute=False)

first_run = True
tm_t = -100
while True:
    while True:
        tm_v, vid_frame = vid.get_frame()
        if vid_frame is None:
            break
        if tm_v > tm_t:
            break
    if vid_frame is None:
        break
    while True:
        tm_t, tof_frame = tof.get_frame()
        if tof_frame is None:
            break
        if tm_t > tm_v:
            break
    if tof_frame is None:
        break
    distance = cv2.normalize(tof_frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    #distance = cv2.medianBlur(distance, 3)
    # confidence= cv2.normalize(confidence, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    colored = cv2.applyColorMap(distance, cv2.COLORMAP_BONE)
    #vid_frame = cv2.rotate(vid_frame, cv2.ROTATE_180)
    colored = cv2.rotate(colored, cv2.ROTATE_90_COUNTERCLOCKWISE)
    colored = cv2.resize(colored,(colored.shape[1]*2,colored.shape[0]*2))
    cv2.imshow("Video", vid_frame)
    cv2.imshow("TOF", colored)
    if first_run:
        first_run = False
        print(np.max(distance), np.min(distance))
        cv2.waitKey(0)
    else:
        res = cv2.waitKey(100)
        if res == ord(' '):
            print(tm_t)
            cv2.waitKey(0)
        if res == ord('q'):
            break