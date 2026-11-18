"""A tiny KNN classifier in numpy, plus save/load.

Written in numpy instead of cv2.ml because OpenCV 5 no longer ships cv2.ml.
"""
import numpy as np

from sign_features import extract_features

EMPTY_WALL = 0
NUM_CLASSES = 6


def save_model(path, features, labels, k):
    np.savez_compressed(path, features=features.astype(np.float32),
                        labels=labels.astype(np.int64), k=k)


def load_model(path):
    data = np.load(path)
    return {"features": data["features"], "labels": data["labels"], "k": int(data["k"])}


def knn_vote(model, x):
    """Label most common among the k stored examples closest to x."""
    distances = ((model["features"] - x) ** 2).sum(axis=1)
    nearest = np.argsort(distances)[:model["k"]]
    votes = np.bincount(model["labels"][nearest], minlength=NUM_CLASSES)
    return int(votes.argmax())


def predict_image(model, image):
    """Classify one BGR image. No sign blob found means empty wall."""
    x = extract_features(image)
    return EMPTY_WALL if x is None else knn_vote(model, x)
