import cv2
import numpy as np

from reader.gstreader import TOFReader

#SPEC = '''filesrc location="/home/phil/footage/tof_stuff/tmp.mkv" ! matroskademux ! video/x-h264 ! decodebin !
#          video/x-raw,format=I420 ! appsink sync=false'''
SPEC = '''filesrc location="/home/phil/footage/tof_stuff/tmp.mkv" ! matroskademux !
          video/x-raw,format=I420 ! appsink sync=false'''

with TOFReader(SPEC, as_uint8=False) as vid:
    while True:
        tm, frame = vid.get_frame()
        if frame is None:
            break
        print(np.max(frame))
        phase = [frame[:,i*240:(i+1)*240].astype("float64") for i in range(4)]
        Q = phase[1] - phase[3]
        I = phase[2] - phase[0]
        distance = np.arctan2(Q, I) % (np.pi * 2)
        confidence = np.sqrt(Q ** 2 + I ** 2)
        distance *= 2/np.pi
        distance[confidence < 100] = 0
        distance = cv2.normalize(distance, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        confidence= cv2.normalize(confidence, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        colored = cv2.applyColorMap(distance,cv2.COLORMAP_JET)
        cv2.imshow("main", colored)
        cv2.imshow("raw", frame)
        cv2.imshow("confidence", confidence)
        key = cv2.waitKey(33)
        if key == ord('q'):
            break
        elif key == ord(' '):
            cv2.waitKey(0)