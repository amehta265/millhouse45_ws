#!/usr/bin/env python3
"""Train the Lab 6 sign classifier.

Steps:
  1. Load every batch under --data and split it into dev / test (grouped, no leakage).
  2. Write the test images to test_set/ in the grader's format (never trained on here).
  3. Pick k with grouped 5-fold cross-validation on dev (augmenting training folds only).
  4. Check generalization to a new batch: train on dev without NEW_BATCH, test on its dev images.
  5. Save sign_knn_dev.npz (dev only, for an honest score on test_set/)
     and sign_knn.npz (all images, the model to submit).

Usage: python3 train.py --data ../../Lab6/Labs_export
"""
import argparse

import cv2
import numpy as np

from augment import augment
from dataset import load_samples, split_dev_test, write_folder
from knn import knn_vote, save_model
from sign_features import extract_features

K_VALUES = [1, 3, 5, 7, 9]
FOLDS = 5
NEW_BATCH = "2026F_Timgs"


def compute_features(samples, num_aug, seed):
    """Add each sample's feature vector and its augmented copies to the sample dict."""
    rng = np.random.default_rng(seed)
    for s in samples:
        image = cv2.imread(s["path"])
        s["x"] = extract_features(image)
        s["aug"] = []
        for _ in range(num_aug):
            aug_image, aug_label = augment(image, s["label"], rng)
            x = extract_features(aug_image)
            if x is not None:
                s["aug"].append((x, aug_label))


def training_rows(samples):
    """Stack originals + augmented copies. Images with no sign blob are skipped
    (at prediction time they are labeled empty wall without the KNN)."""
    rows = [(s["x"], s["label"]) for s in samples if s["x"] is not None]
    rows += [row for s in samples for row in s["aug"]]
    features = np.stack([r[0] for r in rows])
    labels = np.array([r[1] for r in rows])
    return features, labels


def accuracy(train, val, k):
    features, labels = training_rows(train)
    model = {"features": features, "labels": labels, "k": k}
    predictions = [0 if s["x"] is None else knn_vote(model, s["x"]) for s in val]
    return np.mean([p == s["label"] for p, s in zip(predictions, val)])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", required=True, help="Folder holding the image batches")
    parser.add_argument("--test_frac", type=float, default=0.2)
    parser.add_argument("--aug", type=int, default=4, help="Augmented copies per training image")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    samples = load_samples(args.data)
    dev, test = split_dev_test(samples, args.test_frac, args.seed)
    print(f"{len(samples)} images: {len(dev)} dev, {len(test)} test")
    for batch in sorted({s["batch"] for s in samples}):
        n_dev = sum(s["batch"] == batch for s in dev)
        n_test = sum(s["batch"] == batch for s in test)
        print(f"  {batch}: {n_dev} dev, {n_test} test")
    write_folder(test, "test_set")
    print("Wrote test_set/ (score it with model_grader.py and sign_knn_dev.npz)")

    print("Extracting features...")
    compute_features(samples, args.aug, args.seed)

    # Grouped cross-validation: a whole group always lands in one fold.
    groups = sorted({s["group"] for s in dev})
    np.random.default_rng(args.seed).shuffle(groups)
    fold_of = {g: i % FOLDS for i, g in enumerate(groups)}
    print(f"\nGrouped {FOLDS}-fold CV on dev:")
    scores = {}
    for k in K_VALUES:
        fold_scores = [accuracy([s for s in dev if fold_of[s["group"]] != f],
                                [s for s in dev if fold_of[s["group"]] == f], k)
                       for f in range(FOLDS)]
        scores[k] = np.mean(fold_scores)
        print(f"  k={k}: {scores[k]:.3f}")
    best_k = max(K_VALUES, key=lambda k: scores[k])
    print(f"Best k = {best_k}")

    new = [s for s in dev if s["batch"] == NEW_BATCH]
    if new:
        old = [s for s in dev if s["batch"] != NEW_BATCH]
        print(f"New-batch check (train without {NEW_BATCH}, test on its dev images): "
              f"{accuracy(old, new, best_k):.3f}")

    for name, train_set in [("sign_knn_dev.npz", dev), ("sign_knn.npz", samples)]:
        features, labels = training_rows(train_set)
        save_model(name, features, labels, best_k)
        print(f"Saved {name} ({len(labels)} training vectors)")


if __name__ == "__main__":
    main()
