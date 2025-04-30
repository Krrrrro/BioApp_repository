import open3d as o3d
import os
import numpy as np

def compute_dynamic_information(fitness):
    """
    피트니스 스코어 기반으로 동적 Information Matrix 계산
    """
    scale = max(100, min(10000, fitness * 10000))
    return np.eye(6) * scale

def refine_registration(source, target, init_transform, max_correspondence_distance):
    """
    ICP로 쌍별 정합 미세 조정
    """
    reg_icp = o3d.pipelines.registration.registration_icp(
        source, target,
        max_correspondence_distance=max_correspondence_distance,
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
    
    return area, volume

def main():
    # 1. 포인트 클라우드 로드 (5개)
    ply_files = [f for f in os.listdir("filtered") if f.endswith(".ply")]
    ply_files.sort()
    pcds = [o3d.io.read_point_cloud(os.path.join("filtered", f)) for f in ply_files]

    # 2. 노이즈 제거 및 전처리
    voxel_size = 0.0001  # 0.1mm
    avg_nn_distances = []
    for i, pcd in enumerate(pcds):
        pcd = pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=0.7)[0]
        pcd = pcd.voxel_down_sample(voxel_size)
        # 개별 중심화 제거
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.005, max_nn=30))
        # 평균 최근접 이웃 거리 계산
        nn_distances = pcd.compute_nearest_neighbor_distance()
        avg_nn_distance = np.mean(nn_distances)
        print(f"Point cloud {i+1} preprocessed: {len(pcd.points)} points, avg_nn_distance: {avg_nn_distance}")
        pcds[i] = pcd
        avg_nn_distances.append(avg_nn_distance)

    # 3. 쌍별 정합 (FPFH + RANSAC + ICP)
    pairwise_transforms = []
    pairwise_fitness = []
    for i in range(len(pcds) - 1):
        source, target = pcds[i], pcds[i+1]
        # 동적 correspondence 거리 계산
        correspondence_dist_ransac = 10 * max(avg_nn_distances[i], avg_nn_distances[i+1])
        correspondence_dist_icp = 5 * max(avg_nn_distances[i], avg_nn_distances[i+1])
        print(f"Pair {i+1} to {i+2}: RANSAC dist={correspondence_dist_ransac}, ICP dist={correspondence_dist_icp}")
        
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
            max_correspondence_distance=correspondence_dist_ransac,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            ransac_n=4,
            checkers=[
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(correspondence_dist_ransac)
            ],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 0.99)
        )
        # ICP로 미세 조정
        transform, fitness = refine_registration(source, target, reg_ransac.transformation, correspondence_dist_icp)
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
    global_correspondence_dist = 10 * max(avg_nn_distances)
    print(f"Global optimization max_correspondence_distance: {global_correspondence_dist}")
    criteria = o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria()
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=global_correspondence_dist,
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
    final_correspondence_dist = 5 * max(avg_nn_distances)
    print(f"Final ICP max_correspondence_distance: {final_correspondence_dist}")
    combined_pcd = pcds[0]
    for i in range(1, len(pcds)):
        reg_icp = o3d.pipelines.registration.registration_icp(
            pcds[i], combined_pcd,
            max_correspondence_distance=final_correspondence_dist,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100000)
        )
        pcds[i].transform(reg_icp.transformation)
        if(reg_icp.fitness > 0.5):
            combined_pcd += pcds[i]
        print(f"Final ICP for point cloud {i+1}, fitness: {reg_icp.fitness}")
        
    cl, ind = combined_pcd.remove_radius_outlier(nb_points=50, radius=0.005)
    combined_pcd = combined_pcd.select_by_index(ind)

    # 7. 시각화 및 저장
    o3d.visualization.draw_geometries([combined_pcd], window_name="Combined Point Cloud")
    o3d.io.write_point_cloud("combined.ply", combined_pcd)
    print("Saved combined point cloud to combined.ply")

    # 8. Convex Hull 생성 및 부피 계산
    convex_hull, _ = combined_pcd.compute_convex_hull()
    convex_hull.compute_vertex_normals()
    volume = convex_hull.get_volume()
    print(f"Convex Hull Volume: {volume}")

    # 9. 외곽선 및 시각화
    lines = o3d.geometry.LineSet.create_from_triangle_mesh(convex_hull)
    lines.paint_uniform_color([1, 0, 0])
    o3d.visualization.draw_geometries([combined_pcd, lines], window_name="Plant with Convex Hull Outline")

    # 10. Convex Hull 저장
    o3d.io.write_triangle_mesh("convex_hull.ply", convex_hull)
    print("Convex Hull saved as convex_hull.ply")

if __name__ == "__main__":
    main()