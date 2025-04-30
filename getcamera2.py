import math
import json
import uuid

def calculate_camera_positions(posx, posy, yaw, target_x, target_y, target_z, radius_cm, half_circle: bool):
    # 📏 반지름 (단위: cm → m 변환)
    radius = radius_cm / 100.0  # 예: 25cm → 0.25m

    # 📏 카메라 높이 리스트 (단위: cm → m 변환)
    heights_cm = [2]
    heights = [h / 100.0 for h in heights_cm]

    # 각 높이에 대해 수평 반지름 계산 (3D 거리 유지)
    horizontal_radii = [math.sqrt(radius**2 - h**2) if radius > h else 0 for h in heights]

    waypoints = []

    for i in range(0, 81, 20):  # 0, 20, 40, 60, 80
        ind = i / 100  # 예: 0.0, 0.2, ...
        
        # 📐 각도 설정
        if half_circle:
            angles = [60, 70, 80]
        else:
            angles = [300, 290, 280]

        point_data = []
        for h_idx, height in enumerate(heights):
            horizontal_radius = horizontal_radii[h_idx]
            for angle in angles:
                an = angle + 180
                rad = math.radians(an)

                # 카메라 위치 계산
                cam_x = target_x - horizontal_radius * math.cos(rad)
                cam_y = target_y + horizontal_radius * math.sin(rad)
                cam_z = target_z + height

                # 방향 벡터 계산
                dx = target_x - cam_x
                dy = target_y - cam_y
                dz = target_z - cam_z

                yaw_rad = math.atan2(dx, dy)
                horizontal_dist = math.sqrt(dx**2 + dy**2)
                pitch = math.atan2(dz, horizontal_dist)

                angle_print = (angle - 270) % 360
                if angle_print > 180:
                    angle_print -= 360

                targetid = "CB1110" + str(int(i/20) + 1)
                point = {
                    "plant_id": targetid,
                    "armendpoint_x": cam_x - ind,
                    "armendpoint_y": cam_y,
                    "armendpoint_z": cam_z,
                    "cam_pitch": pitch * -1,
                    "cam_yaw": yaw_rad * -1,
                    "sensor": 2
                }
                point_data.append(point)
            angles.reverse()

        waypoint = {
            "mode": 1,
            "goal_x": posx - ind,
            "goal_y": posy,
            "goal_yaw": yaw,
            "plants": point_data
        }
        waypoints.append(waypoint)

    outdata = {
        "header": {
            "version": 0,
            "type": 1210
        },
        "body": {
            "robot_id": 1,
            "command_id": str(uuid.uuid4()),
            "order_mode": 1,
            "way_points": waypoints
        }
    }

    json.dump(outdata, open("output.json", "w"), indent=4)

# 테스트 호출
posx = 0.7
posy = 0
yaw = 0
t_x = 0.7
t_y = 0.8
t_z = 0.85
radian = 28

calculate_camera_positions(posx, posy, yaw, t_x, t_y, t_z, radian, True)
