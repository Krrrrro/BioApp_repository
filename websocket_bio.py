import websocket
import threading
import time
import json
import os
import re
from ftplib import FTP
import model.new_filtering as mod
from websocket_manager import manager
from collections import defaultdict
from shared_state import message_buffer
from flask import Flask, send_from_directory
from tkinter import messagebox
import xml.etree.ElementTree as ET
import socket
CONFIG_FILE = "config.xml"
DATA_DIR = "data"
HTTP_DATA_DIR = "result"
ftp = ""
# Flask 앱 생성
broker = ""
app = Flask(__name__)


@app.route("/<path:filename>")
def download_file(filename):
    return send_from_directory(HTTP_DATA_DIR, filename, as_attachment=True)

def run_http_server():
    print("🌐 HTTP 파일 서버 시작: http://<IP>:8000/files/<파일명>")
    app.run(host="0.0.0.0", port=8000)


def download_ftp_file(ftp_path, filename):
    def ftp_download():
        try:
            ftp = FTP("10.0.0.2")
            ftp.login(user="amrlabs", passwd="dev4321")
            ftp.cwd(ftp_path)
            local_path = os.path.join(DATA_DIR, filename)
            with open(local_path, "wb") as f:
                ftp.retrbinary("RETR " + filename, f.write)
            ftp.quit()
            print(f"Downloaded {filename} to {local_path}")
        except Exception as e:
            print("FTP download error:", e)
    threading.Thread(target=ftp_download, daemon=True).start()

def separate_header_message(data: bytes):
    try:
        header = data[:20]
        message_bytes = data[20:]
        message = json.loads(message_bytes.decode('utf-8').strip())
        return header.decode("utf-8").rstrip("\x00"), message
    except Exception as e:
        print("JSON 디코딩 에러:", e, data)
        return None, None

def on_message(ws, message):

    header, message = separate_header_message(message)
    # type = message["header"]["type"]
    try:
        if header == "1202":
            command = message["body"]["command_id"]
            print("왔다", re.search(r'-\d+$', command))
            if re.search(r'-\d+$', command) == None and message["body"]["state"] == 1:
                threading.Thread(target=mod.run, daemon=True).start()
        elif (header == "1301" or header == "1302"):
            body = message["body"]
            sensor = body["target"]["sensor"]
            if sensor == 2 :
                if header == "1301":            
                    target_id = body["target"]["target_id"]
                    message_buffer[target_id]["sensor"].append(body)
                    message_buffer[target_id]["robot_id"] = body["robot_id"]
                elif header == "1302":
                    path = body["path"]
                    if path.endswith(".ply"):
                        print('🟢 FTP 다운로드 시작')
                        ftp_path = "/".join(path.split("/")[:-1])
                        filename = path.split("/")[-1]
                        download_ftp_file(ftp_path, filename)
        
    except Exception as e:
        print("메시지 처리 중 에러:", e)

def on_error(ws, error):
    print(f"🛑 WebSocket 에러 발생: {error}")

def on_close(ws, close_status_code, close_msg):
    print("🔌 WebSocket 연결 종료")

def on_open(ws):
    manager.set_ws(ws)
    print("🔗 WebSocket 연결 성공")
    # threading.Thread(target=mod.run, daemon=True).start()

def run_websocket():
    ws_url = f"ws://{broker}:6000/Server"

    while True:
        print(f"🔄 WebSocket {ws_url}로 연결 시도 중...")
        try:
            ws_app = websocket.WebSocketApp(
                ws_url,
                on_open=on_open,
                on_message=on_message,
                on_error=on_error,
                on_close=on_close
            )
            ws_app.run_forever()
        except Exception as e:
            print("WebSocket run_forever 중 예외:", e)

        print("⏳ 재연결까지 5초 대기...")
        time.sleep(5)
def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # 가짜 주소로 연결 시도해서 로컬 IP 알아냄
        s.connect(("8.8.8.8", 80))  # Google DNS
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip
def set_data_format(success, filename, weight, area, sensors, targetid, robot_id) -> bytes:
    # current_path = os.getcwd()
    # current_folder_name = os.path.basename(current_path)
    # path = os.path.join(current_folder_name, path).replace("\\", "/")
    ip = get_local_ip()
    key = '7777'
    value = f'''{{
        "header": {{
            "version": 0,
            "type": 7777
        }},
        "body": {{
            "robot_id": 1,
            "success" : {success},
            "path" : "http://{ip}:8000/{filename}",
            "weight": {weight},
            "sensors" : {json.dumps(sensors)},
            "area" : {area},
            "targetid" : "{targetid}",
            "robot_id" : {robot_id}
        }}
    }}'''
    header = bytearray(20)
    str_value = value.encode('utf-8')
    result = bytearray(len(header) + len(str_value))
    result[20:] = str_value
    key_bytes = key.encode('utf-8')
    header[:len(key_bytes)] = key_bytes[:20]
    result[:20] = header
    return bytes(result)

def average_sensor_data(sensor_list):
    temp = sum(d["temperature"] for d in sensor_list) / len(sensor_list)
    humid = sum(d["humidity"] for d in sensor_list) / len(sensor_list)
    illum = sum(d["illuminance"] for d in sensor_list) / len(sensor_list)
    co2 = sum(d["co2"] for d in sensor_list) / len(sensor_list)
    return {"temperature": temp, "humidity": humid, "illuminance": illum, "co2": co2}

def send_message(success, path, weight, area):
    # base = os.getcwd()
    # file_path = os.path.join(base, "data", path)
    print(message_buffer)
    max_target_id = max(message_buffer.items(), key=lambda item: len(item[1]["sensor"]))[0]
    most_sensor_data = message_buffer[max_target_id]["sensor"]
    robot_id = message_buffer[max_target_id]["robot_id"]
    datas = average_sensor_data(most_sensor_data)
    msg = set_data_format(success, path, weight, area, datas, max_target_id, robot_id)
    del message_buffer[max_target_id]
    manager.send(msg)

def wait_until_connected(timeout=5.0):
    start = time.time()
    while time.time() - start < timeout:
        if manager.is_connected():
            return True
        time.sleep(0.1)
    return False

def read_config():
    try:
        global ftp
        global broker
        tree = ET.parse(CONFIG_FILE)
        root = tree.getroot()
        ftp = root.find("FTP").text
        broker = root.find("broker").text
        
        return True
    except FileNotFoundError:
        print("Error", f"Failed to parse config: {e}")
        return False
    except Exception as e:
        print("Error", f"Failed to parse config: {e}")
        return False

def main() :
    print("🚀 WebSocket 시작")

    threading.Thread(target=run_websocket, daemon=True).start()
    threading.Thread(target=run_http_server, daemon=True).start()
    while True:
        time.sleep(1)

if __name__ == "__main__":
    if(read_config()):
        main()
