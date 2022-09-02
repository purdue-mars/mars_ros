from PIL import Image  # (pip install Pillow)
import numpy as np  # (pip install numpy)
from skimage import measure  # (pip install scikit-image)
from shapely.geometry import Polygon, MultiPolygon  # (pip install Shapely)
import numpy as np
import cv2


def create_sub_masks(mask_image_np):
    mask_image = Image.fromarray(mask_image_np)
    width, height = mask_image.size

    # Initialize a dictionary of sub-masks indexed by RGB colors
    sub_masks = {}
    for x in range(width):
        for y in range(height):
            # Get the RGB values of the pixel
            pixel = mask_image.getpixel((x, y))[:3]

            # If the pixel is not black...
            if pixel != (0, 0, 0):
                # Check to see if we've created a sub-mask...
                pixel_str = str(pixel)
                sub_mask = sub_masks.get(pixel_str)
                if sub_mask is None:
                    # Create a sub-mask (one bit per pixel) and add to the dictionary
                    # Note: we add 1 pixel of padding in each direction
                    # because the contours module doesn't handle cases
                    # where pixels bleed to the edge of the image
                    sub_masks[pixel_str] = Image.new("1", (width + 2, height + 2))

                # Set the pixel value to 1 (default is 0), accounting for padding
                sub_masks[pixel_str].putpixel((x + 1, y + 1), 1)

    return sub_masks


def create_sub_mask_annotation(
    sub_mask, image_id, category_id, annotation_id, is_crowd
):
    # Find contours (boundary lines) around each sub-mask
    # Note: there could be multiple contours if the object
    # is partially occluded. (E.g. an elephant behind a tree)
    contours = measure.find_contours(sub_mask, 0.5, positive_orientation="low")

    segmentations = []
    polygons = []
    for contour in contours:
        # Flip from (row, col) representation to (x, y)
        # and subtract the padding pixel
        for i in range(len(contour)):
            row, col = contour[i]
            contour[i] = (col - 1, row - 1)

        # Make a polygon and simplify it
        poly = Polygon(contour)
        poly = poly.simplify(1.0, preserve_topology=False)
        polygons.append(poly)
        segmentation = np.array(poly.exterior.coords).ravel().tolist()
        segmentations.append(segmentation)

    # Combine the polygons to calculate the bounding box and area
    multi_poly = MultiPolygon(polygons)
    x, y, max_x, max_y = multi_poly.bounds
    width = max_x - x
    height = max_y - y
    bbox = (x, y, width, height)
    area = multi_poly.area

    annotation = {
        "segmentation": segmentations,
        "iscrowd": is_crowd,
        "image_id": image_id,
        "category_id": category_id,
        "id": annotation_id,
        "bbox": bbox,
        "area": area,
    }

    return annotation


def hsv_mask(frame, threshold):
    """Convert numpy RGB frame to HSV and and threshold image to given color range in HSV.
    Arguments:
    frame -- image to generate mask in np array format
    threshold -- hsv color range consisting of tuple of np arrays (lower color bound, upper color bound)

    """

    frame = frame[:, :, ::-1]  # to BGR
    blur = cv2.blur(frame, (5, 5))
    blur0 = cv2.medianBlur(blur, 5)
    blur1 = cv2.GaussianBlur(blur0, (5, 5), 0)
    blur2 = cv2.bilateralFilter(blur1, 9, 75, 75)
    hsvFrame = cv2.cvtColor(blur2, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsvFrame, *threshold)
    out = cv2.bitwise_and(frame, frame, mask=mask)
    out[out != 0] = 255
    return out
