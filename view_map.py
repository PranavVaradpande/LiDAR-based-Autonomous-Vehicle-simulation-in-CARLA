import open3d as o3d
import numpy as np
import os

def main():
    filename = "carla_point_cloud_map.ply"
    
    if not os.path.exists(filename):
        print(f"Error: Could not find '{filename}'")
        return

    print(f"Loading 3D map from {filename}...")
    pcd = o3d.io.read_point_cloud(filename)
    points = np.asarray(pcd.points)
    print(f"Loaded {len(points)} points. Applying custom Cyberpunk colormap...")

    # --- CUSTOM COLORMAP (Green -> Magenta -> Blue) ---
    z_vals = points[:, 2]
    
    # Use percentiles to clip extreme outliers (like a single point flying way too high)
    # This ensures the color gradient stretches nicely over the actual map structure
    z_min = np.percentile(z_vals, 2)
    z_max = np.percentile(z_vals, 98)
    
    # Normalize Z values between 0.0 and 1.0 based on those percentiles
    z_norm = np.clip((z_vals - z_min) / (z_max - z_min + 1e-6), 0.0, 1.0)
    
    colors = np.zeros((points.shape[0], 3))
    
    # Fast vectorized color assignment
    # Lower half: Transition from Neon Green (0, 1, 0) to Magenta (1, 0, 1)
    mask_low = z_norm < 0.5
    t_low = z_norm[mask_low] * 2.0
    colors[mask_low, 0] = t_low          # Red increases
    colors[mask_low, 1] = 1.0 - t_low    # Green decreases
    colors[mask_low, 2] = t_low          # Blue increases
    
    # Upper half: Transition from Magenta (1, 0, 1) to Light Blue (0, 0.5, 1.0)
    mask_high = z_norm >= 0.5
    t_high = (z_norm[mask_high] - 0.5) * 2.0
    colors[mask_high, 0] = 1.0 - t_high       # Red decreases
    colors[mask_high, 1] = t_high * 0.5       # Green increases slightly for lighter blue
    colors[mask_high, 2] = 1.0                # Blue stays maxed
    
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # --- SETUP VISUALIZER ---
    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name="CARLA Cyberpunk LiDAR Map", 
        width=1280, 
        height=720,
        left=50, top=50
    )
    
    # Match the dark navy background from the image
    vis.get_render_option().background_color = np.asarray([0.07, 0.07, 0.12])
    vis.get_render_option().point_size = 2.0
    
    vis.add_geometry(pcd)
    
    # Set a nice starting 3D isometric angle automatically
    view_control = vis.get_view_control()
    view_control.set_front([-0.6, -0.6, 0.5])
    view_control.set_up([0, 0, 1])
    
    print("Opening 3D Viewer! Use mouse to rotate, right-click to zoom.")
    vis.run()
    vis.destroy_window()

if __name__ == "__main__":
    main()