import numpy as np
import open3d as o3d
import trimesh

pcd = o3d.io.read_point_cloud("result/" + "08b5f3d71-8750-4236-a024-b92189989cba11.ply")
points = np.asarray(pcd.points)

# 삼각형 메쉬로 변환
mesh = trimesh.convex.convex_hull(points)

# 부피 계산
volume = mesh.volume
print(f"📐 부피: {volume * 1e6:.2f} cm³")