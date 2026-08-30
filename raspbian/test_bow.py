import operator

import numpy as np

import orb_slam3_py as op
from collections import defaultdict
import math
n_words_total = 1_000_000
FNAME="/data/trips/trip_095/atlas.osa"
atlas = op.load_atlas(FNAME)
maps = atlas.get_all_maps()
dct = defaultdict(int)
kf_count = 0
mp_count = 0
desc_count = 0
for m in maps:
    mp_count += len(m.get_all_map_points())
    for kf in m.get_all_keyframes():
        kf_count += 1
        desc_count += len(kf.get_descriptors())
        for key in kf.bowVec:
            dct[key] += 1

lst = sorted(list(dct.items()), key=operator.itemgetter(1), reverse=True)
used = np.array(sorted(dct.values(), reverse=True))
n_used = len(used)
n_unused = n_words_total - n_used

total_hits = used.sum()
probs = used / total_hits
entropy = -np.sum(probs * np.log2(probs))
norm_entropy = entropy / math.log2(n_words_total)

print(entropy, norm_entropy)
print(f"Kfs, {kf_count}")
print(f"MPs, {mp_count}")
print(f"Descs, {desc_count}")
print(f"desc/kf, {desc_count/kf_count}")
print("Count: ", len(lst))
print(list(y for x,y in lst)[:100])


