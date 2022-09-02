"""
Script to determine color range in HSV of a selected region
"""

import numpy as np
import cv2 as cv
import argparse

parser = argparse.ArgumentParser()

parser.add_argument("-image_path", type=str)
args = vars(parser.parse_args())

img = cv.imread(args["image_path"])
print(img)
print("Select color range to threshold")
r = cv.selectROI(img)
cv.waitKey()

im_crop = img[int(r[1]) : int(r[1] + r[3]), int(r[0]) : int(r[0] + r[2])]
im_crop_hsv = cv.cvtColor(im_crop, cv.COLOR_BGR2HSV)

lower_bound = np.array(
    [im_crop_hsv[..., 0].min(), im_crop_hsv[..., 1].min(), im_crop_hsv[..., 2].min()]
)
upper_bound = np.array(
    [im_crop_hsv[..., 0].max(), im_crop_hsv[..., 1].max(), im_crop_hsv[..., 2].max()]
)

print(
    f"Lower and upper bound of color range in cropped selection: {lower_bound}, {upper_bound}"
)

blur = cv.blur(img, (5, 5))
blur0 = cv.medianBlur(blur, 5)
blur1 = cv.GaussianBlur(blur0, (5, 5), 0)
blur2 = cv.bilateralFilter(blur1, 9, 75, 75)

cv.imshow("image with preprocessing", blur2)
cv.waitKey()

hsv = cv.cvtColor(blur2, cv.COLOR_BGR2HSV)

mask = cv.inRange(hsv, lower_bound, upper_bound)
res = cv.bitwise_and(img, img, mask=mask)

cv.imshow("image with mask", res)
cv.waitKey()
