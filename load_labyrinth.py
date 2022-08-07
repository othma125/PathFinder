from os import listdir
from os.path import isfile, join

import numpy as np
from PIL import Image


def color_map(color: list) -> int:
    color = list(color)
    if color == [255, 255, 255, 255]:
        return 1
    elif color == [0, 0, 0, 255]:
        return 0
    elif color == [0, 0, 255, 255]:
        return 2
    elif color == [255, 0, 0, 255]:
        return 3
    else:
        return -1


# Load a png to a numpy array
def load_png(filename: str):
    rgb = np.array(Image.open(filename))
    return np.array([[color_map(rgb[x, y]) for y in range(rgb.shape[1])] for x in range(rgb.shape[0])])


# Get all png files in a directory
def get_png_files(directory: str) -> list:
    return [f for f in listdir(directory) if isfile(join(directory, f)) and f.endswith(".png")]
