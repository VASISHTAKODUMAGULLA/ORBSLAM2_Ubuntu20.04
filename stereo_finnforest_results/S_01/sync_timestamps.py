#!/usr/bin/env python3
import numpy as np

# Adjust these filenames if needed
gt_file     = "GT_S01_tum.txt"
est_file    = "CameraTrajectory.txt"
output_file = "CameraTrajectory_sync.txt"

# Load ground‑truth (time + pose) and estimated (time + pose)
gt = np.loadtxt(gt_file)
est = np.loadtxt(est_file)

# Replace the first column (timestamps) of est with GT timestamps
est[:, 0] = gt[:, 0]

# Save out the synchronized trajectory in TUM format
# fmt="%.9f" ensures high‑precision for all eight columns
np.savetxt(output_file, est, fmt="%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f")

print(f"Synchronized trajectory saved as {output_file}")
