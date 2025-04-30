# websocket_manager.py
import websocket

class WebSocketManager:
    def __init__(self):
        self.ws = None

    def set_ws(self, ws):
        self.ws = ws

    def is_connected(self):
        return self.ws and self.ws.sock and self.ws.sock.connected

    def send(self, msg):
        if self.is_connected():
            self.ws.send(msg)
            print("✅ 메시지 전송 완료")
        else:
            print("❌ WebSocket이 연결되지 않았습니다.")

manager = WebSocketManager()
