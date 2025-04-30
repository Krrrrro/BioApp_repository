import math
import json
import uuid
def calculate_camera_positions(posx, posy, yaw, target_x, target_y, target_z, radius_cm, half_circle: bool):

    
    # 📏 반지름 (단위: cm → m 변환)
    radius = radius_cm / 100.0  # → 0.26 m

    # 📏 카메라 높이 리스트 (단위: cm → m 변환)
    heights_cm = [2]
    # heights_cm = [2]
    heights = [h / 100.0 for h in heights_cm]

    
    waypoints = []

    for i in range(0, 81, 20) :
        ind = i / 100
        # 📐 각도 범위 설정
        if half_circle:
            # 12시(270°) → 6시(90°), 시계 방향: 12시 → 1시 → 2시 → 3시 → 4시 → 5시 → 6시
            angles = [60, 70, 80]
        else:
            # 12시(270°) → 6시(90°), 반시계 방향: 12시 → 11시 → 10시 → 9시 → 8시 → 7시 → 6시
            angles = [300, 290, 280]
        point_data = []
        for height in heights:
            for angle in angles:
                an = angle + 180
                # ⭕ 원형 궤적을 위한 각도 변환
                rad = math.radians(an)

                # 카메라 위치 계산 (12시 = X+ / 3시 = X-)
                cam_x = target_x - radius * math.cos(rad)  # X 반전: cos 증가 시 X 감소
                cam_y = target_y + radius * math.sin(rad)
                cam_z = target_z + height

                # 방향 벡터
                dx = target_x - cam_x
                dy = target_y - cam_y
                dz = target_z - cam_z

                # Yaw (수평 방향) - 라디안으로 출력
                yaw_rad = math.atan2(dx, dy)

                # Pitch (수직 방향) - 라디안으로 출력
                horizontal_dist = math.sqrt(dx**2 + dy**2)
                pitch = math.atan2(dz, horizontal_dist)

                # 출력 각도 보정: 기준을 12시 방향이 0°가 되도록 설정
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
        waypoint =  {
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
posx = 1.7
posy = -1.57
yaw = 0
t_x = 1.86
t_y = -0.91
t_z = 1.23
radian = 25

calculate_camera_positions(posx,posy,yaw, t_x, t_y, t_z, radian, True)  # 재배대가 왼쪽에 잇을때
# calculate_camera_positions(1.54, -0.93, 0.8, 26, False)  # 오른쪽에 있을때