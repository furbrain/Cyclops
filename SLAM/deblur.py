import collections
from pathlib import Path

import cv2
import numpy as np
from matplotlib import pyplot as plt
from skimage.restoration import wiener, unsupervised_wiener

WORKING_DIR: Path = Path("/home/phil/footage/bpotw/")

FNAME = WORKING_DIR / "test.jpg"

img = cv2.imread(str(FNAME), cv2.IMREAD_GRAYSCALE)
img = cv2.resize(img, (480,640))
#img = cv2.normalize(img, )
def make_psf(angle, len):
    psf = np.zeros((len+2,len+2))
    cv2.ellipse(psf, (len//2+1, len//2+1), (len//2, 0),angle,0,360, (255,),cv2.FILLED)
    psf /= np.sum(psf)
    return psf



results = collections.OrderedDict()
for i in np.linspace(6, 13, 8):
    psf = make_psf(15,int(i))
    results[i] = wiener(img, psf, 1, clip=False)
fig, ax = plt.subplots(ncols = 3, nrows = 3)
ax[0,0].imshow(img[400:, 50:250], cmap="gray")
ax[0,0].set_title("original")
for count, (i, deconv) in enumerate(results.items()):
    x = (count+1)//3
    y = (count+1)% 3
    ax[x,y].imshow(deconv[400:, 50:250], cmap="gray")
    ax[x,y].set_title(f"{i:4g}")
    ax[x,y].set_xlabel(f"{deconv.var():6g}")
    ax[x,y].set_xticks([])
plt.show()