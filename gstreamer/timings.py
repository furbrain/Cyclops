#!/usr/bin/env python3

import numpy as np
import timeit

def get_upper_byte_rounded(d: np.array):
    return ((d+128) // 256).astype("int8")


def mungulator(d: np.ndarray):
    d = d.astype("int32")
    phase = [d[:,i*240:(i+1)*240] for i in range(4)]
    Q = phase[1] - phase[3]
    I = phase[2] - phase[0]
    magnitude = Q**2 + I**2
    bad_pixels = magnitude < 300
    stack = np.dstack((Q,I))
    mx = np.maximum(np.abs(Q), np.abs(I))
    multiplier = ((2<<14) / mx).astype("int16")
    multiplier = np.minimum(multiplier, 255)
    Q = get_upper_byte_rounded(Q * multiplier)
    I = get_upper_byte_rounded(Q * multiplier)
    Q[bad_pixels] = 0
    I[bad_pixels] = 0
    output = np.hstack((Q,I))
    return output
    
data = np.random.randint(-16384,16384, (180,960)).astype("int16")
print(timeit.timeit(lambda : mungulator(data), number=1000))
