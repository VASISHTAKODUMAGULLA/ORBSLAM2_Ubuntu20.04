import matplotlib.pyplot as plt
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface

# Load trajectories
gt = file_interface.read_tum_trajectory_file("Grond_truth.txt")
est = file_interface.read_tum_trajectory_file("CameraTrajectory_sync.txt")

# Create side-by-side plots
fig, axs = plt.subplots(1, 2, figsize=(12, 6))

# Plot settings
line_width = 1.0  # thinner lines
alpha_val = 0.9   # transparency if needed

# Ground Truth
axs[0].plot(gt.positions_xyz[:, 0], gt.positions_xyz[:, 2],
            label="GT", color="blue", linewidth=line_width, alpha=alpha_val)
axs[0].set_title("Ground Truth")
axs[0].set_xlabel("X [m]")
axs[0].set_ylabel("Z [m]")
axs[0].axis('equal')
axs[0].grid(True)

# Estimated Trajectory
axs[1].plot(est.positions_xyz[:, 0], est.positions_xyz[:, 2],
            label="Estimate", color="red", linewidth=line_width, alpha=alpha_val)
axs[1].set_title("SLAM Output")
axs[1].set_xlabel("X [m]")
axs[1].set_ylabel("Z [m]")
axs[1].axis('equal')
axs[1].grid(True)

plt.tight_layout()
plt.savefig("traj_S_03_xz_side_by_side_cleaned.png", dpi=300)
plt.show()
