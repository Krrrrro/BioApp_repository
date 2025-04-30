import open3d as o3d
import os
import numpy as np

def compute_dynamic_information(fitness):
    """
    피트니스 스코어 기반으로 동적 Information Matrix 계산
    """
    scale = max(100, min(10000, fitness * 10000))
    return np.eye(6) * scale

def refine_registration(source, target, init_transform, voxel_size):
    """
    ICP로 쌍별 정합 미세 조정
    """
    reg_icp = o3d.pipelines.registration.registration_icp(
        source, target,
        max_correspondence_distance=voxel_size * 2,
        init=init_transform,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=10000)
    )
    return reg_icp.transformation, reg_icp.fitness

def create_mesh_and_calculate_area_volume(pcd):
    
    # 비유한 점 제거
    pcd.remove_non_finite_points()
    if len(pcd.points) == 0:
        print("오류: 유효한 포인트가 없습니다. PLY 파일을 확인하세요.")
        return None, None
    
    # 법선 벡터 계산 (볼 피봇 알고리즘을 위해 필요)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.0001, max_nn=20))
    pcd.orient_normals_consistent_tangent_plane(k=20)
    
    # 볼 피봇 알고리즘으로 메쉬 생성
    # radii는 연결할 최대 거리를 조정합니다 (단위: 미터)
    radii = [0.005, 0.005, 0.008]  # 작은 물체(5cm)에 맞게 설정, 필요 시 조정
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))
    
    # 메쉬가 비어 있는지 확인
    if len(mesh.vertices) == 0 or len(mesh.triangles) == 0:
        print("오류: 생성된 메쉬가 비어 있습니다. radii 값을 조정하거나 입력 데이터를 확인하세요.")
        return None, None
    
    # 메쉬 정제
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()
    
    # 법선 벡터 계산 (STL 저장을 위해 필요)
    mesh.compute_vertex_normals()
    
    # 메쉬 시각화
    o3d.visualization.draw_geometries([mesh], window_name=f"Mesh Visualization: 'aaaa")
    
    # 메쉬가 밀폐된 상태인지 확인
    if not mesh.is_watertight():
        temp_path = "temp_mesh.stl"
        o3d.io.write_triangle_mesh(temp_path, mesh)
        volume = None
    else:
        # 메쉬의 부피 계산
        volume = mesh.get_volume()
    
    # 메쉬의 표면 면적 계산
    area = mesh.get_surface_area()
    
    # 최종 메쉬 저장
    # output_path = os.path.splitext(ply_path)[0] + "_processed.stl"
    # if len(mesh.vertices) > 0 and len(mesh.triangles) > 0:
    #     success = o3d.io.write_triangle_mesh(output_path, mesh)
    #     if success:
    #         file_size = os.path.getsize(output_path)
    # else:
    #     print("오류: 메쉬에 유효한 데이터가 없어 저장하지 않습니다.")
    
    return area, volume

def main():
    # 1. 포인트 클라우드 로드 (5개)
    # filtered 폴더에서 .ply 파일 리스트 가져오기
    ply_files = [f for f in os.listdir("filtered") if f.endswith(".ply")]

    # 파일 이름을 정렬하고 읽기
    ply_files.sort()  # 파일명이 숫자 순이라면 정렬 필요
    pcds = [o3d.io.read_point_cloud(os.path.join("filtered", f)) for f in ply_files]

    # 2. 노이즈 제거 및 전처리
    voxel_size = 0.0001  # 1mm
    for i, pcd in enumerate(pcds):
        pcd = pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=0.7)[0]
        pcd = pcd.voxel_down_sample(voxel_size)
        centroid = np.mean(np.asarray(pcd.points), axis=0)
        pcd.translate(-centroid)
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.005, max_nn=30))
        pcds[i] = pcd
        print(f"Point cloud {i+1} preprocessed: {len(pcd.points)} points")


    # 3. 쌍별 정합 (FPFH + RANSAC + ICP)
    pairwise_transforms = []
    pairwise_fitness = []
    for i in range(len(pcds) - 1):
        source, target = pcds[i], pcds[i+1]
        # FPFH 특징 추출
        radius_normal = 0.005
        radius_feature = 0.01
        source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            source, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            target, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

        # RANSAC으로 초기 정렬
        reg_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source, target, source_fpfh, target_fpfh, mutual_filter=True,
            max_correspondence_distance=0.002,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            ransac_n=4,
            checkers=[
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(0.002)
            ],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 0.99)
        )
        # ICP로 미세 조정
        transform, fitness = refine_registration(source, target, reg_ransac.transformation, voxel_size)
        pairwise_transforms.append(transform)
        pairwise_fitness.append(fitness)
        print(f"Pair {i+1} to {i+2} registered, fitness: {fitness}")

    # 4. 글로벌 정합 (Pose Graph Optimization)

    pose_graph = o3d.pipelines.registration.PoseGraph()
    accumulated_transform = np.eye(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(accumulated_transform))

    for i in range(len(pcds) - 1):
        transform = pairwise_transforms[i]
        fitness = pairwise_fitness[i]
        information = compute_dynamic_information(fitness)
        edge = o3d.pipelines.registration.PoseGraphEdge(
            source_node_id=i,
            target_node_id=i+1,
            transformation=transform,
            information=information
        )
        pose_graph.edges.append(edge)
        accumulated_transform = accumulated_transform @ transform
        pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(accumulated_transform))
        print(f"Added edge {i} to {i+1}")



    # 글로벌 최적화

    criteria = o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria()
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=0.002,
        edge_prune_threshold=0.25,
        reference_node=0
    )

    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        criteria,
        option
    )
    print("Global optimization completed")

    # 5. 최적화된 변환 적용
    for i, pcd in enumerate(pcds):
        pcd.transform(pose_graph.nodes[i].pose)
        print(f"Transformed point cloud {i+1}")

    # 6. 최종 ICP로 미세 조정
    combined_pcd = pcds[0]
    for i in range(1, len(pcds)):
        reg_icp = o3d.pipelines.registration.registration_icp(
            pcds[i], combined_pcd,
            max_correspondence_distance=voxel_size * 2,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=10000)
        )

        pcds[i].transform(reg_icp.transformation)
        combined_pcd += pcds[i]
        print(f"Final ICP for point cloud {i+1}, fitness: {reg_icp.fitness}")


    # 7. 시각화 및 저장
    o3d.visualization.draw_geometries([combined_pcd], window_name="Combined Point Cloud")
    o3d.io.write_point_cloud("combined.ply", combined_pcd)
    print("Saved combined point cloud to combined.ply")

    # 2. Convex Hull 생성
    convex_hull, _ = combined_pcd.compute_convex_hull()
    convex_hull.compute_vertex_normals()  # 법선 계산으로 시각화 개선

    # 3. 부피 계산
    volume = convex_hull.get_volume()
    print(f"Convex Hull Volume: {volume}")

    # 4. 외곽선 및 부피 시각화
    # Convex Hull을 와이어프레임으로 표시하기 위해 선 집합(LineSet) 생성
    lines = o3d.geometry.LineSet.create_from_triangle_mesh(convex_hull)
    lines.paint_uniform_color([1, 0, 0])  # 빨간색 외곽선

    # 원본 포인트 클라우드와 함께 시각화
    o3d.visualization.draw_geometries([combined_pcd, lines], window_name="Plant with Convex Hull Outline")

    # 5. Convex Hull 저장 (선택적)
    o3d.io.write_triangle_mesh("convex_hull.ply", convex_hull)
    print("Convex Hull saved as convex_hull.ply")
    
main()