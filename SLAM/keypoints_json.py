import cv2

FIELDS = ("pt", "size", "response", "angle", "octave", "class_id")


def kp_as_dict(obj):
    dct = {x:getattr(obj,x) for x in FIELDS}
    return dct

def kp_from_dict(dct):
    obj = cv2.KeyPoint()
    apply_dict(obj, dct)
    return obj

def apply_dict(obj, dct):
    for x in FIELDS:
        setattr(obj,x,dct[x])

def register_keypoint_pickles():
    cv2.KeyPoint.__setstate__ = apply_dict
    cv2.KeyPoint.__getstate__ = kp_as_dict
