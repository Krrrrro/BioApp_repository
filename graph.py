import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def visualize_camera_orbit(target_x, target_y, target_z, radius_cm, half_circle=True):
    # 카메라 거리 및 높이
    radius = radius_cm / 100.0
    heights_cm = [2, 6, 10, 14]
    heights = [h / 100.0 for h in heights_cm]
    horizontal_radii = [math.sqrt(radius**2 - h**2) if radius > h else 0 for h in heights]

    # 각도 설정
    if half_circle:
        angles = [60, 70, 80, 90, 100, 110, 120]
    else:
        angles = [300, 290, 280, 270, 260, 250, 240]

    cam_positions = []
    for h_idx, height in enumerate(heights):
        horizontal_radius = horizontal_radii[h_idx]
        for angle in angles:
            an = angle + 180
            rad = math.radians(an)

            cam_x = target_x - horizontal_radius * math.cos(rad)
            cam_y = target_y + horizontal_radius * math.sin(rad)
            cam_z = target_z + height

            cam_positions.append((cam_x, cam_y, cam_z))

    # 시각화
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 카메라 점들
    xs, ys, zs = zip(*cam_positions)
    ax.plot(xs, ys, zs, 'o-', c='blue', label='Camera Path')

    # 타겟 표시
    ax.scatter([target_x], [target_y], [target_z], c='red', s=100, label='Target')

    # 카메라-타겟 거리선
    for x, y, z in cam_positions:
        ax.plot([x, target_x], [y, target_y], [z, target_z], 'gray', linestyle='--', linewidth=1)

    # 그래프 라벨
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("3D Camera Orbit Around Target")
    ax.legend()

    plt.show()

# 사용 예시
visualize_camera_orbit(target_x=1.86, target_y=-0.91, target_z=1.23, radius_cm=20, half_circle=True)
