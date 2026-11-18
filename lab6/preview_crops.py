#!/usr/bin/env python3
"""Save a contact sheet showing which blob the preprocessing picks in each image.

Green box = light (arrow) blob, blue box = pink blob, dark image = nothing found.
Usage: python3 preview_crops.py --folder test_set --out preview.jpg
"""
import argparse
import csv
import os

import cv2
import numpy as np

from sign_features import find_sign

COLUMNS = 10


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder", required=True, help="Folder with images and labels.txt")
    parser.add_argument("--out", default="preview.jpg")
    args = parser.parse_args()

    with open(os.path.join(args.folder, "labels.txt")) as f:
        rows = sorted((int(r[1]), r[0].strip()) for r in csv.reader(f) if r)
    tiles = []
    for label, name in rows:
        image = cv2.imread(os.path.join(args.folder, name + ".png"))
        found = find_sign(image)
        if found is None:
            image = image // 3
        else:
            _, is_pink, (x, y, w, h) = found
            color = (255, 0, 0) if is_pink else (0, 255, 0)
            cv2.rectangle(image, (x, y), (x + w, y + h), color, 3)
        cv2.putText(image, str(label), (8, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (255, 255, 255), 3)
        tiles.append(cv2.resize(image, (205, 154)))
    tiles += [np.zeros_like(tiles[0])] * (-len(tiles) % COLUMNS)
    grid = np.vstack([np.hstack(tiles[i:i + COLUMNS]) for i in range(0, len(tiles), COLUMNS)])
    cv2.imwrite(args.out, grid)
    print(f"Saved {args.out}")


if __name__ == "__main__":
    main()
