import open3d as o3d
import os
import numpy as np
import uuid
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

def main():
    # 1. 포인트 클라우드 로드
    ply_files = [f for f in os.listdir("filtered") if f.endswith(".ply")]
    ply_files.sort()
    pcds = [o3d.io.read_point_cloud(os.path.join("filtered", f)) for f in ply_files]

    # 2. 노이즈 제거 및 전처리
    voxel_size = 0.0002
    avg_nn_distances = []
    for i, pcd in enumerate(pcds):
        pcd = pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=0.5)[0]
        pcd = pcd.voxel_down_sample(voxel_size)
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
        nn_distances = pcd.compute_nearest_neighbor_distance()
        avg_nn_distance = np.mean(nn_distances)
        print(f"Point cloud {i+1} preprocessed: {len(pcd.points)} points, avg_nn_distance: {avg_nn_distance}")
        pcds[i] = pcd
        avg_nn_distances.append(avg_nn_distance)

    # 3. 쌍별 정합 (FPFH + RANSAC + ICP) - 뒤의 3개 이웃까지만 수행
    pairwise_edges = []
    neighbor_limit = 3
    for i in range(len(pcds)):
        for j in range(i + 1, min(i + 1 + neighbor_limit, len(pcds))):
            source, target = pcds[i], pcds[j]
            correspondence_dist_ransac = 8 * max(avg_nn_distances[i], avg_nn_distances[j])
            correspondence_dist_icp = 4 * max(avg_nn_distances[i], avg_nn_distances[j])
            print(f"Pair {i+1} to {j+1}: RANSAC dist={correspondence_dist_ransac}, ICP dist={correspondence_dist_icp}")

            # FPFH 특징 추출
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
                criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(500000, 0.99)
            )
            # ICP로 미세 조정
            transform, fitness = refine_registration(source, target, reg_ransac.transformation, correspondence_dist_icp)

            if fitness > 0.7:
                pairwise_edges.append((i, j, transform, fitness))
                print(f"Pair {i+1} to {j+1} registered, fitness: {fitness}")
            else:
                print(f"Pair {i+1} to {j+1} skipped due to low fitness: {fitness}")

    # 4. 글로벌 정합 (Pose Graph Optimization)
    pose_graph = o3d.pipelines.registration.PoseGraph()
    # 모든 포인트 클라우드에 대해 노드 생성 (IndexError 방지)
    for i in range(len(pcds)):
        pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(np.eye(4)))

    # 유효한 엣지 추가
    for source_id, target_id, transform, fitness in pairwise_edges:
        information = compute_dynamic_information(fitness)
        edge = o3d.pipelines.registration.PoseGraphEdge(
            source_node_id=source_id,
            target_node_id=target_id,
            transformation=transform,
            information=information
        )
        pose_graph.edges.append(edge)
        print(f"Added edge {source_id} to {target_id}, fitness: {fitness}")

    # 글로벌 최적화
    global_correspondence_dist = 10 * max(avg_nn_distances)
    print(f"Global optimization max_correspondence_distance: {global_correspondence_dist}")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=global_correspondence_dist,
        edge_prune_threshold=0.25,
        reference_node=0
    )
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option
    )
    print("Global optimization completed")

    # 5. 최적화된 변환 적용
    for i, pcd in enumerate(pcds):
        pcd.transform(pose_graph.nodes[i].pose)
        print(f"Transformed point cloud {i+1}")
        # print(f"Transformed point cloud {i+1}")

    # 6. 포인트 클라우드 결합 및 후처리
    combined_pcd = pcds[0]
    final_correspondence_dist = 4 * max(avg_nn_distances)
    for i in range(1, len(pcds)):
        reg_icp = o3d.pipelines.registration.registration_icp(
            pcds[i], combined_pcd,
            max_correspondence_distance=final_correspondence_dist,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100000)
        )
        pcds[i].transform(reg_icp.transformation)
        if reg_icp.fitness > 0.7:
            combined_pcd += pcds[i]
        print(f"Final ICP for point cloud {i+1}, fitness: {reg_icp.fitness}")

    combined_pcd, _ = combined_pcd.remove_radius_outlier(nb_points=50, radius=0.005)

    # 7. 시각화 및 저장
    # o3d.visualization.draw_geometries([combined_pcd], window_name="Combined Point Cloud")
    result_name = str(uuid.uuid4()) + '.ply'
    save_path = os.path.join("result", result_name)
    o3d.io.write_point_cloud(save_path, combined_pcd, write_ascii=False, compressed=False)
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
    return result_name, combined_pcd, volume

if __name__ == "__main__":
    main()