Lab 6 sign classifier (KNN) - Mehta & Huntley

Run the grader from this folder:
    pip install -r requirements.txt
    python3 model_grader.py --data_path ./val_dataset --model_path ./sign_knn.npz

Files
    model_grader.py      grader with initialize_model() and predict() filled in
    sign_knn.npz         trained model (stored training feature vectors, labels and k)
    sign_features.py     preprocessing: color masks -> largest sign blob -> square crop -> features
    knn.py               KNN in numpy (no cv2.ml, which OpenCV 5 removed), save/load, predict
    train.py             training script; dataset.py and augment.py are its helpers

How it works
    Pink signs (do not enter, stop, goal) are found with an HSV mask; light arrows
    with a Lab mask (bright and on the green side of a*). The largest sign-like blob is
    cropped, padded to a square and shrunk to 20x20. That shape plus a color flag, size,
    fill and aspect ratio form the feature vector. If no blob is found the image is
    labeled 0 (empty wall). Otherwise a KNN over the stored training vectors votes.

Retraining (needs the image batches; any folder tree containing labels.txt files):
    python3 train.py --data <folder with the image batches>
