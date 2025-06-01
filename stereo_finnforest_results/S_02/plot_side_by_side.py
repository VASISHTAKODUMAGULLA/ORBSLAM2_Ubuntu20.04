
















# import matplotlib.pyplot as plt
# from evo.core.trajectory import PoseTrajectory3D
# from evo.tools import file_interface

# # ✅ Correct KITTI loader function
# gt = file_interface.read_kitti_poses_file("GroundTruth.txt")
# est = file_interface.read_kitti_poses_file("Cameratrajectory.txt")

# # Create side-by-side plots
# fig, axs = plt.subplots(1, 2, figsize=(12, 6))

# # Plot settings
# line_width = 1.0
# alpha_val = 0.9

# # Ground Truth
# axs[0].plot(gt.positions_xyz[:, 0], gt.positions_xyz[:, 2],
#             label="GT", color="blue", linewidth=line_width, alpha=alpha_val)
# axs[0].set_title("Ground Truth (XZ)")
# axs[0].set_xlabel("X [m]")
# axs[0].set_ylabel("Z [m]")
# axs[0].axis('equal')
# axs[0].grid(True)

# # Estimated Trajectory
# axs[1].plot(est.positions_xyz[:, 0], est.positions_xyz[:, 2],
#             label="Estimate", color="red", linewidth=line_width, alpha=alpha_val)
# axs[1].set_title("SLAM Output (XZ)")
# axs[1].set_xlabel("X [m]")
# axs[1].set_ylabel("Z [m]")
# axs[1].axis('equal')
# axs[1].grid(True)

# plt.tight_layout()
# plt.savefig("traj_xz_side_by_side_kitti.png", dpi=300)
# plt.show()






import matplotlib.pyplot as plt
from evo.tools import file_interface
from evo.core import sync
from evo.core.metrics import PoseRelation, APE, RPE
from evo.core.trajectory import PoseTrajectory3D
# from evo.core.transformations import se3_log
import numpy as np

# --- Load KITTI trajectories ---
# gt = file_interface.read_kitti_poses_file("GroundTruth.txt")
# est = file_interface.read_kitti_poses_file("Cameratrajectory.txt")



# from evo.core.trajectory import PoseTrajectory3D

# # --- Load KITTI trajectories and wrap in PoseTrajectory3D ---
# gt_raw = file_interface.read_kitti_poses_file("GroundTruth.txt")
# est_raw = file_interface.read_kitti_poses_file("Cameratrajectory.txt")

# gt = PoseTrajectory3D(poses_se3=gt_raw)
# est = PoseTrajectory3D(poses_se3=est_raw)




from evo.tools import file_interface

# Load and wrap correctly
gt_path = file_interface.read_kitti_poses_file("GroundTruth.txt")
est_path = file_interface.read_kitti_poses_file("Cameratrajectory.txt")

gt = PoseTrajectory3D(poses_se3=gt_path.poses_se3)
est = PoseTrajectory3D(poses_se3=est_path.poses_se3)



# --- Synchronize poses (by index, not timestamp) ---
# Useful if you know both have equal sampling
gt_aligned, est_aligned = sync.associate_trajectories(gt, est)

# --- Plot side-by-side (XZ view) ---
fig, axs = plt.subplots(1, 2, figsize=(12, 6))
line_width = 1.0
alpha_val = 0.9

axs[0].plot(gt_aligned.positions_xyz[:, 0], gt_aligned.positions_xyz[:, 2],
            label="GT", color="blue", linewidth=line_width, alpha=alpha_val)
axs[0].set_title("Ground Truth (XZ)")
axs[0].set_xlabel("X [m]")
axs[0].set_ylabel("Z [m]")
axs[0].axis('equal')
axs[0].grid(True)

axs[1].plot(est_aligned.positions_xyz[:, 0], est_aligned.positions_xyz[:, 2],
            label="Estimate", color="red", linewidth=line_width, alpha=alpha_val)
axs[1].set_title("SLAM Output (XZ)")
axs[1].set_xlabel("X [m]")
axs[1].set_ylabel("Z [m]")
axs[1].axis('equal')
axs[1].grid(True)

plt.tight_layout()
plt.savefig("traj_xz_side_by_side_kitti.png", dpi=300)
plt.show()

# --- Compute ATE ---
ape_metric = APE(PoseRelation.translation_part)
ape_metric.process_data((gt_aligned, est_aligned))
ape_rmse = ape_metric.get_all_statistics()["rmse"]
print(f"\n✅ ATE RMSE: {ape_rmse:.4f} m")

# --- Compute RPE (delta=1 frame) ---
rpe_metric = RPE(PoseRelation.translation_part, delta=1, delta_unit="frames")
rpe_metric.process_data((gt_aligned, est_aligned))
rpe_rmse = rpe_metric.get_all_statistics()["rmse"]
print(f"✅ RPE RMSE (Δ=1): {rpe_rmse:.4f} m")

# --- Optional: Save APE error plot ---
plt.figure()
plt.plot(ape_metric.error)
plt.title("APE Error Per Frame")
plt.xlabel("Frame")
plt.ylabel("Translation Error [m]")
plt.grid(True)
plt.savefig("ape_error_plot.png", dpi=300)
plt.show()

# --- Optional: Save RPE error plot ---
plt.figure()
plt.plot(rpe_metric.error)
plt.title("RPE Error Per Frame (Δ=1)")
plt.xlabel("Frame")
plt.ylabel("Translation Error [m]")
plt.grid(True)
plt.savefig("rpe_error_plot.png", dpi=300)
plt.show()


