import open3d as o3d
import time
from pathlib import Path

# 设置点云文件夹路径
# pcd_folder = Path("/home/roborock/IsaacLab/pointcloud/NAV_binId10_pcd")

# # 获取所有 PCD 文件并按名字排序
# pcd_files = sorted(pcd_folder.glob("*.pcd"))

# vis = o3d.visualization.Visualizer()
# vis.create_window("PCD Viewer")
# ctr = vis.get_view_control()
# param = o3d.io.read_pinhole_camera_parameters("/home/roborock/IsaacLab/pointcloud/viewpoint.json")


# added = False
# frame = 0
# for frame, file in enumerate(pcd_files[100:1500], 1):
#     print(f"Frame {frame}")
#     cloud = o3d.io.read_point_cloud(str(file))
#     if len(cloud.points) == 0:
#         print(f"跳过空帧: {file.name}")
#         continue
#     vis.clear_geometries()
#     vis.add_geometry(cloud)
#     ctr.convert_from_pinhole_camera_parameters(param)
#     vis.poll_events()
#     vis.update_renderer()

#     # 应用保存的相机参数
   

#     time.sleep(0.1)

# vis.destroy_window()



# cloud = o3d.io.read_point_cloud('/home/roborock/IsaacLab/pointcloud/NAV_binId10_pcd/frame_1300.pcd')
# vis = o3d.visualization.Visualizer()
# vis.create_window()
# vis.add_geometry(cloud)

# vis.run()  # 这里可以调整视角
# param = vis.get_view_control().convert_to_pinhole_camera_parameters()
# o3d.io.write_pinhole_camera_parameters("viewpoint.json", param)
# vis.destroy_window()
# import open3d as o3d
# import numpy as np

# # 读取 TXT 文件
# points = np.loadtxt("/home/roborock/下载/frame(3).txt")  # shape: (N,3)

# # 创建 Open3D 点云对象
# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(points)

# # 可视化
# o3d.visualization.draw_geometries([pcd])
