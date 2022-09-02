from torch import nn
import cv2
import numpy as np


def init_weights(m):
    if isinstance(m, nn.Linear):
        nn.init.xavier_uniform_(m.weight)
        m.bias.data.fill_(0.01)
    elif isinstance(m, nn.Conv2d):
        nn.init.normal_(m.weight.data, 0.0, 0.02)
    elif isinstance(m, nn.BatchNorm2d):
        nn.init.normal_(m.weight.data, 1.0, 0.02)
        nn.init.constant_(m.bias.data, 0)


def measure_darkness(im: cv2.Mat):
    num_dark_px = np.count_nonzero(im == 0)
    num_total_px = im.shape[0] * im.shape[1]
    return num_dark_px / num_total_px


def obj_center_crop(im: cv2.Mat, desired: tuple = None):
    inds = np.argwhere(im != 0)
    max_p = np.max(inds, axis=0)
    min_p = np.min(inds, axis=0)
    centered = im[min_p[0] : max_p[0], min_p[1] : max_p[1]]
    if desired is not None:
        delta_w = desired[1] - centered.shape[1]
        delta_h = desired[0] - centered.shape[0]
        top, bottom = max(delta_h // 2, 0), max(delta_h - (delta_h // 2), 0)
        left, right = max(delta_w // 2, 0), max(delta_w - (delta_w // 2), 0)
        color = [0, 0, 0]
        centered = cv2.copyMakeBorder(
            centered, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color
        )
    center = (centered.shape[0] / 2, centered.shape[1] / 2)
    x = center[1] - desired[1] / 2
    y = center[0] - desired[0] / 2

    centered = centered[int(y) : int(y + desired[0]), int(x) : int(x + desired[1])]
    return centered


def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result
