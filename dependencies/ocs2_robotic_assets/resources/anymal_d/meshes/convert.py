#!/usr/bin/env python3

import os
import shutil
import cv2
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from PIL import Image

import argparse


def main():
    parser = argparse.ArgumentParser(description="100th anniversary anymal")
    parser.add_argument(
        "--visualize", action="store_true", help="Visualize color changes"
    )
    args = parser.parse_args()

    matplotlib.use("TkAgg")
    items = ["bottom_shell.jpg", "top_shell.jpg", "face_shell.jpg"]

    for item in items:
        name, extension = item.split(".")
        copy = name + "_copy" + "." + extension
        if os.path.exists(copy) is False:
            shutil.copy(item, copy)

    for item in items:
        orig = item
        name, extension = item.split(".")
        item = name + "_copy" + "." + extension

        RED_RANGE_LOW = (100, 5, 5)  # rgb
        RED_RANGE_HIGH = (255, 110, 100)  # rgb

        # https://www.anybotics.com/news/anybotics-celebrates-100th-anymal-milestones/
        YELLOW_COLLOR = (228, 169, 49)  # rgb

        print("Processing " + item)
        img = np.array(Image.open(item))

        img_inv = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        mask = np.bool_(cv2.inRange(img_inv, RED_RANGE_LOW[::-1], RED_RANGE_HIGH[::-1]))

        modified = np.copy(img)
        modified[mask] = YELLOW_COLLOR

        if args.visualize:
            fig, ax = plt.subplots(1, 2)
            ax[0].imshow(img)
            ax[0].set_title("Original")
            ax[1].imshow(modified)
            ax[1].set_title("Modified")
            plt.show()

        # Store the image
        Image.fromarray(modified).save(orig)

    print("Done")


if __name__ == "__main__":
    main()
