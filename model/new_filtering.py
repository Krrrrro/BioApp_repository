import open3d as o3d
import numpy as np
import colorsys
import os
import model.new_4 as reg
# import model.test as predict
from websocket_bio import wait_until_connected, send_message
def is_plant_green(rgb):
    """
    RGB 색상이 식물 초록색 범위인지 확인.
    조건:
    - HSV: Hue 40~140도, Saturation >= 10%, Value >= 20%
    - G >= 60
    """
    r, g, b = rgb
    # G < 60이면 초록색으로 간주하지 않음
    # if g < 30:
    #     return False
    # HSV 변환
    r_norm, g_norm, b_norm = [x / 255.0 for x in rgb]
    h, s, v = colorsys.rgb_to_hsv(r_norm, g_norm, b_norm)
    h = h * 360
    # Hue(색상)이 대략 50~140이면 초록색 계열
    # Saturation(채도)가 0.1이하 인것은 제외(너무 흐릿한건 제외)
    # Value(명도) 0.2이하 인것은 제외(너무 어두운것은 제외)
    return 50 <= h <= 140 and s >= 0.15 and v >= 0.15

def filter_green_points(ply_path, output_path="green_filtered.ply", visualize=True):
    print(f"📂 Loading PLY file: {ply_path}")
    pcd = o3d.io.read_point_cloud(ply_path)

    if not pcd.has_points():
        print("❌ Point cloud is empty!")
        return

    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors) * 255.0

    # ROI 필터
    z_mask = points[:, 2] < 0.28
    x_mask = np.logical_and(points[:, 0] > -0.1, points[:, 0] < 0.1)
    combined_mask = np.logical_and(z_mask, x_mask)
    points = points[combined_mask]
    colors = colors[combined_mask]


    if len(points) == 0:
        print("⚠️ No points remain after ROI filtering.")
        return

    # 초록색 필터링
    green_mask = np.array([is_plant_green(rgb) for rgb in colors])
    green_points = points[green_mask]
    green_colors = colors[green_mask] / 255.0

    if len(green_points) == 0:
        print("⚠️ No green points found.")
        return

    green_pcd = o3d.geometry.PointCloud()
    green_pcd.points = o3d.utility.Vector3dVector(green_points)
    green_pcd.colors = o3d.utility.Vector3dVector(green_colors)

    # 🧹 반경 기반 노이즈 제거
    green_pcd, _ = green_pcd.remove_radius_outlier(nb_points=16, radius=0.01)

    # 💠 클러스터링
    labels = np.array(green_pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=False))
    if labels.max() < 0:
        print("⚠️ DBSCAN 결과 클러스터 없음")
        return

    all_points = np.asarray(green_pcd.points)
    all_colors = np.asarray(green_pcd.colors)
    # 가장 0,0,0에 가까운 클러스터 선택
    best_label = -1
    min_distance = float('inf')

    for label in np.unique(labels):
        cluster_points = all_points[labels == label]
        centroid = cluster_points.mean(axis=0)
        dist = np.linalg.norm(centroid)

        if dist < min_distance:
            min_distance = dist
            best_label = label

    best_mask = labels == best_label
    final_points = all_points[best_mask]
    final_colors = all_colors[best_mask]
    final_pcd = o3d.geometry.PointCloud()
    final_pcd.points = o3d.utility.Vector3dVector(final_points)
    final_pcd.colors = o3d.utility.Vector3dVector(final_colors)

    o3d.io.write_point_cloud(output_path, final_pcd)
    print(f"✅ Saved closest-cluster point cloud to: {output_path}")

    # 시각화
    print("Visualizing green points...")
    # o3d.visualization.draw_geometries([green_pcd], window_name="Green Filtered Point Cloud")

def filtering(ply_path, index):
    # 예시 PLY 파일 경로 (사용자가 제공해야 함)
    output_path = f"filtered/filtered_{index}.ply"
    
    if not os.path.exists(ply_path):
        print(f"Error: {ply_path} does not exist!")
        return
    
    filter_green_points(ply_path, output_path)

def clear_data_folder(folder):
    try:
        for filename in os.listdir(folder):
            file_path = os.path.join(folder, filename)
            if os.path.isfile(file_path):
                os.remove(file_path)
            elif os.path.isdir(file_path):
                # 폴더일 경우 삭제하지 않음 (필요하면 shutil.rmtree 사용)
                continue
        print("All files in 'data' folder deleted.")
    except FileNotFoundError:
        print("'data' folder does not exist.")
    except Exception as e:
        print(f"Error deleting files: {e}")
        
def run():
    try:
            directory = "data"
            processed_folder = "filtered"
            filename = ''
            clear_data_folder(processed_folder)
            index = 0
            for file in os.listdir(directory):
                if file.endswith(".ply"):
                    index += 1 
                    filename = file
                    filtering(os.path.join(directory, file), index)

            clear_data_folder(directory)
            path, combined_pcd, area = reg.main()
            area = area * 1000000
            weight = area / 8.5
            success = 1
            if wait_until_connected(timeout=10):
                send_message(success, path, weight, area)
            else:
                print("❌ WebSocket 연결 실패로 메시지 전송 안 함")
    except Exception as e:
        print("💥 run() 함수에서 에러 발생:", e)



if __name__ == "__main__":
    run()
    # print("success : ", success, "path : ", path)
    # if wait_until_connected(timeout=10):
    #     send_message(success, path, weight, area)
    # else:
    #     print("❌ WebSocket 연결 실패로 메시지 전송 안 함")