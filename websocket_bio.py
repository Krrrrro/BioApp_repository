# -*- coding: utf-8 -*-
"""
BioApp 실시간 파이프라인 (WebSocket + FTP + Flask 파일서버)
- 안정성 보강: WS ping/pong, TCP keepalive, Flask reloader off, thread-safe buffer
- 기존 import 전부 포함 + 보강용 import 포함
"""

import websocket
from websocket import ABNF
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
import sys
import platform
import logging

# ---------------------------
# 전역 상수/전역 상태
# ---------------------------
CONFIG_FILE = "config.xml"
DATA_DIR = "data"
HTTP_DATA_DIR = "result"

# XML에서 읽어올 항목 (기본값은 빈 문자열)
ftp = ""
broker = ""

# thread-safe 접근을 위한 락 (message_buffer는 외부 모듈에서 제공)
_lock = threading.Lock()

# WebSocket 디버그 로그 (필요 시 True)
websocket.enableTrace(False)

# Flask 앱
app = Flask(__name__)


# ---------------------------
# 유틸리티/공용 함수
# ---------------------------
def get_local_ip():
    """로컬 IP 조회(UDP dummy connect 방식)"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip


def separate_header_message(data: bytes):
    """
    서버 포맷: [20바이트 헤더][JSON 텍스트]
    """
    try:
        header = data[:20]
        message_bytes = data[20:]
        message = json.loads(message_bytes.decode('utf-8').strip())
        return header.decode("utf-8").rstrip("\x00"), message
    except Exception as e:
        print("JSON 디코딩 에러:", e)
        return None, None


def average_sensor_data(sensor_list):
    """센서 데이터 평균"""
    temp = sum(d["temperature"] for d in sensor_list) / len(sensor_list)
    humid = sum(d["humidity"] for d in sensor_list) / len(sensor_list)
    illum = sum(d["illuminance"] for d in sensor_list) / len(sensor_list)
    co2 = sum(d["co2"] for d in sensor_list) / len(sensor_list)
    return {"temperature": temp, "humidity": humid, "illuminance": illum, "co2": co2}


def set_data_format(success, filename, weight, area, sensors, targetid, robot_id) -> bytes:
    """
    바이너리(20바이트 헤더 + UTF-8 JSON)로 포맷팅
    - key는 20바이트 헤더에 '7777' 로 채움
    """
    ip = get_local_ip()
    key = '7777'
    value = f'''{{
        "header": {{
            "version": 0,
            "type": 7777
        }},
        "body": {{
            "robot_id": 1,
            "success" : {str(bool(success)).lower()},
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


def wait_until_connected(timeout=5.0):
    """manager.is_connected() 대기"""
    start = time.time()
    while time.time() - start < timeout:
        if manager.is_connected():
            return True
        time.sleep(0.1)
    return False


def _sockopts_for_keepalive():
    """
    TCP keepalive 활성화 (가능한 OS에서 세부값도 세팅)
    - Linux/macOS: TCP_KEEPIDLE/TCP_KEEPINTVL/TCP_KEEPCNT
    - Windows: SO_KEEPALIVE만 (세부는 제한)
    """
    opts = [(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)]
    if hasattr(socket, "TCP_KEEPIDLE"):
        opts += [
            (socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 60),
            (socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 10),
            (socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3),
        ]
    return tuple(opts)


# ---------------------------
# Flask (파일 서버)
# ---------------------------
@app.route("/<path:filename>")
def download_file(filename):
    return send_from_directory(HTTP_DATA_DIR, filename, as_attachment=True)


def run_http_server():
    print("🌐 HTTP 파일 서버 시작: http://<이PC IP>:8000/<파일명>")
    # reloader를 끄지 않으면 백그라운드에서 프로세스 복제가 발생할 수 있음
    app.run(host="0.0.0.0", port=8000, threaded=True, use_reloader=False)


# ---------------------------
# FTP 다운로드 (비동기)
# ---------------------------
def download_ftp_file(ftp_path, filename):
    def ftp_download():
        try:
            # XML에서 읽은 값(ftp 변수)을 활용하려면 서버 주소만 ftp 변수에서 가져오도록 바꿔도 됨
            # 여기서는 IP를 고정 사용하던 기존 코드를 유지
            f = FTP("10.0.0.2")
            f.login(user="amrlabs", passwd="dev4321")
            f.cwd(ftp_path)
            os.makedirs(DATA_DIR, exist_ok=True)
            local_path = os.path.join(DATA_DIR, filename)
            with open(local_path, "wb") as out:
                f.retrbinary("RETR " + filename, out.write)
            f.quit()
            print(f"📥 Downloaded {filename} → {local_path}")
        except Exception as e:
            print("FTP download error:", e)

    threading.Thread(target=ftp_download, daemon=True).start()


# ---------------------------
# WebSocket 이벤트 핸들러
# ---------------------------
def on_data(ws, data, data_type, cont):
    """바이너리 프레임 직접 처리"""
    if data_type == ABNF.OPCODE_BINARY:
        header, payload = separate_header_message(data)
        if header is None:
            return
        try:
            if header == "1202":
                command = payload["body"]["command_id"]
                print("1202 수신, 커맨드 패턴:", re.search(r'-\d+$', command))
                if re.search(r'-\d+$', command) is None and payload["body"]["state"] == 1:
                    threading.Thread(target=mod.run, daemon=True).start()

            elif header in ("1301", "1302"):
                body = payload["body"]
                sensor = body["target"]["sensor"]
                if sensor == 2:
                    if header == "1301":
                        target_id = body["target"]["target_id"]
                        with _lock:
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
    else:
        # 텍스트 프레임 로그
        try:
            txt = data.decode("utf-8") if isinstance(data, (bytes, bytearray)) else data
            if txt and len(txt) < 200:
                print("ℹ️ text frame:", txt)
        except Exception:
            pass


def on_message(ws, message):
    """혹시 텍스트로 JSON이 오는 환경을 위한 보조 처리"""
    if isinstance(message, (bytes, bytearray)):
        on_data(ws, message, ABNF.OPCODE_BINARY, False)
        return
    try:
        obj = json.loads(message)
        print("ℹ️ text json:", obj.get("header") or obj)
    except Exception:
        print("ℹ️ text:", str(message)[:200])


def on_ping(ws, msg):
    print("📡 ping")


def on_pong(ws, msg):
    print("📡 pong")


def on_error(ws, error):
    print(f"🛑 WebSocket 에러 발생:", error)


def on_close(ws, code, msg):
    print(f"🔌 WebSocket 연결 종료 (code={code}, msg={msg})")


def on_open(ws):
    manager.set_ws(ws)
    print("🔗 WebSocket 연결 성공")


# ---------------------------
# WebSocket 실행 루프
# ---------------------------
def run_websocket():
    ws_url = f"ws://{broker}:6000/Server"
    sockopts = _sockopts_for_keepalive()

    while True:
        print(f"🔄 WebSocket {ws_url}로 연결 시도 중...")
        try:
            ws_app = websocket.WebSocketApp(
                ws_url,
                on_open=on_open,
                on_message=on_message,
                on_data=on_data,
                on_error=on_error,
                on_close=on_close,
                on_ping=on_ping,
                on_pong=on_pong,
            )

            # ping_interval로 정기 핑, ping_timeout 내 pong 없으면 끊고 아래 루프에서 재연결
            ws_app.run_forever(
                ping_interval=30,
                ping_timeout=10,
                ping_payload="keepalive",
                sockopt=sockopts,
                http_proxy_host=None,
                http_proxy_port=None,
            )
        except Exception as e:
            print("WebSocket run_forever 중 예외:", e)

        print("⏳ 재연결까지 5초 대기...")
        time.sleep(5)


# ---------------------------
# 메시지 전송/집계
# ---------------------------
def send_message(success, path, weight, area):
    """
    message_buffer에서 센서 데이터가 가장 많이 쌓인 target_id를 선택해 응답 전송
    """
    print(message_buffer)
    with _lock:
        max_target_id = max(message_buffer.items(), key=lambda item: len(item[1]["sensor"]))[0]
        most_sensor_data = message_buffer[max_target_id]["sensor"]
        robot_id = message_buffer[max_target_id]["robot_id"]

    datas = average_sensor_data(most_sensor_data)
    msg = set_data_format(success, path, weight, area, datas, max_target_id, robot_id)

    with _lock:
        del message_buffer[max_target_id]

    manager.send(msg)


# ---------------------------
# 설정 읽기
# ---------------------------
def read_config():
    """
    config.xml 예)
    <config>
        <FTP>10.0.0.2</FTP>
        <broker>127.0.0.1</broker>
    </config>
    """
    global ftp, broker
    try:
        tree = ET.parse(CONFIG_FILE)
        root = tree.getroot()
        ftp = root.findtext("FTP", default="10.0.0.2")
        broker = root.findtext("broker", default="127.0.0.1")
        print(f"⚙️ 설정: FTP={ftp}, broker={broker}")
        return True
    except FileNotFoundError as e:
        print("Error", f"Failed to parse config: {e}")
        return False
    except Exception as e:
        print("Error", f"Failed to parse config: {e}")
        return False


# ---------------------------
# 엔트리 포인트
# ---------------------------
def main():
    print("🚀 WebSocket & HTTP 서버 시작")
    # 파일 서버 & WS를 데몬 스레드로 실행
    threading.Thread(target=run_websocket, daemon=True).start()
    threading.Thread(target=run_http_server, daemon=True).start()

    # 메인 스레드는 단순 대기
    while True:
        time.sleep(1)


if __name__ == "__main__":
    # 로그 설정(선택)
    logging.basicConfig(
        filename="bioapp_ws.log",
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )
    if read_config():
        main()
    else:
        print("❌ 설정 파일 로드 실패. 프로그램을 종료합니다.")
