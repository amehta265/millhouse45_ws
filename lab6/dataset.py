"""Load the image batches and split them into dev and test sets without leakage."""
import csv
import glob
import os
import shutil

import numpy as np

RUN_LENGTH = 5   # consecutive shots of the same sign are near-copies; keep them together

# Near-identical photos found across two batches. Both copies always stay in dev.
NEAR_DUPLICATES = [("2025F_imgs/099", "2025F_Gimgs/083"),
                   ("2025F_imgs/100", "2025F_Gimgs/101"),
                   ("2025F_imgs/124", "2025F_Gimgs/126")]


def load_samples(data_root):
    """Find every labels.txt under data_root. Each folder holding one is a batch."""
    samples = []
    for labels_file in sorted(glob.glob(os.path.join(data_root, "**", "labels.txt"), recursive=True)):
        folder = os.path.dirname(labels_file)
        batch = os.path.basename(folder)
        with open(labels_file) as f:
            for row in csv.reader(f):
                if row:
                    name = row[0].strip()
                    samples.append({"id": f"{batch}/{name}", "batch": batch, "name": name,
                                    "path": os.path.join(folder, name + ".png"),
                                    "label": int(row[1])})
    return samples


def assign_groups(samples):
    """Group each batch+class into runs of RUN_LENGTH consecutive filenames."""
    by_stratum = {}
    for s in samples:
        by_stratum.setdefault((s["batch"], s["label"]), []).append(s)
    for (batch, label), members in by_stratum.items():
        members.sort(key=lambda s: s["name"])
        for i, s in enumerate(members):
            s["group"] = f"{batch}/{label}/{i // RUN_LENGTH}"
    # A near-duplicate pair joins one group, so the pair is never split apart.
    by_id = {s["id"]: s for s in samples}
    for a, b in NEAR_DUPLICATES:
        if a in by_id and b in by_id:
            by_id[b]["group"] = by_id[a]["group"]
    return by_stratum


def split_dev_test(samples, test_frac, seed):
    """Hold out about test_frac of the groups in every batch+class as the test set."""
    by_stratum = assign_groups(samples)
    duplicate_ids = {i for pair in NEAR_DUPLICATES for i in pair}
    rng = np.random.default_rng(seed)
    test_groups = set()
    for members in by_stratum.values():
        groups = sorted({s["group"] for s in members if s["id"] not in duplicate_ids})
        rng.shuffle(groups)
        test_groups.update(groups[:max(1, round(test_frac * len(groups)))])
    dev = [s for s in samples if s["group"] not in test_groups]
    test = [s for s in samples if s["group"] in test_groups]
    return dev, test


def write_folder(samples, out_dir):
    """Copy samples into one folder in the grader's format (images + labels.txt)."""
    if os.path.isdir(out_dir):
        shutil.rmtree(out_dir)
    os.makedirs(out_dir)
    with open(os.path.join(out_dir, "labels.txt"), "w") as f:
        for s in samples:
            new_name = f"{s['batch']}_{s['name']}"
            shutil.copy(s["path"], os.path.join(out_dir, new_name + ".png"))
            f.write(f"{new_name}, {s['label']}\n")
