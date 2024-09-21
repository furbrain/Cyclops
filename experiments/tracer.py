import re
from collections import defaultdict
import numpy as np

TIME_PATTERN = r"time = (\d+)"
ELEMENT_PATTERN = r'element = "(.*?)"'
times = defaultdict(list)
with open("sharky") as f:
    for line in f:
        if "proctime" in line:
            tm = re.search(TIME_PATTERN, line).group(1)
            el = re.search(ELEMENT_PATTERN, line).group(1)
            times[el].append(tm)

        if "cpuusage" in line:
            #print(line)
            pass

        if "buffer" in line:
            print(line)

for key, values in times.items():
    arr = np.array(values,dtype="float64")/1_000_000
    print(f"{key}: Max: {np.max(arr)}, Mean: {np.mean(arr)}")