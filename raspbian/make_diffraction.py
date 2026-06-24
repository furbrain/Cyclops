import cv2
import numpy as np

GRID_SIZE = 50
NUM_PYRAMIDS = 5

img = np.zeros((GRID_SIZE, GRID_SIZE), np.uint64)
for i in range(GRID_SIZE):
    for j in range(GRID_SIZE):
        img[i, j] = min(i,GRID_SIZE - i, j, GRID_SIZE - j)
max_pixel = np.max(img)
img *= 255
img //= max_pixel
img  = np.hstack([img]*NUM_PYRAMIDS)
img = np.vstack([img]*NUM_PYRAMIDS)
cv2.imshow('img', img.astype(np.uint8))
cv2.waitKey(0)
cv2.imwrite("diffraction.png", img)
cv2.destroyAllWindows()