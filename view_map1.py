import open3d as o3d

print("Loading map...")
# Load the saved point cloud
pcd = o3d.io.read_point_cloud("carla_point_cloud_map.ply")

print(f"Map loaded! It contains {len(pcd.points)} points.")
# Open the visualizer
o3d.visualization.draw_geometries([pcd])