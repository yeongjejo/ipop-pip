import json
import threading
import websocket

from data_manager import DataManager
from sensor.acc import Acc
from sensor.gyro import Gyro
from sensor.quaternion import Quaternion
from sensor.sensor_part import SensorPart


SERVER_URL = "ws://192.168.201.100:8084"

class ResetClient(threading.Thread):
    def __init__(self):
        super().__init__()
        self._running = True
        
    def stop(self):
        self._running = False  # 스레드 종료 플래그 설정
        self.join()  # 스레드가 종료될 때까지 대기

    def run(self):
        # WebSocket 클라이언트 연결 설정
        ws = websocket.WebSocketApp(SERVER_URL,
                                    on_open=self.on_open,
                                    on_message=self.on_message,
                                    on_error=self.on_error,
                                    on_close=self.on_close)

        # 연결을 유지하며 서버와 통신
        ws.run_forever()
        
        
    def on_open(self, ws):
        print("웹소켓 서버에 연결되었습니다.")

        DataManager().ws = ws
        # # 문자열 데이터 전송
        # message = "reset"  # 서버에서 처리할 메시지
        # ws.send(message)
        
        # print(f"서버로 전송한 메시지: {message}")


    def on_message(self, ws, message):
        # print(f"서버로부터 받은 메시지: {message}")
        try:
            # JSON 문자열을 딕셔너리로 변환
            message_dict = json.loads(message)
            # print("11111111111111111")
            # 딕셔너리 항목 순회
            for key, value in message_dict.items():
                # print(f"{key}: {value}")
                # print(f"{key}, {SensorPart(int(key))}")
                gyro = Gyro(0,0,0)
                mag = Gyro(0,0,0)
                acc = Acc(value['eulerX'], value['eulerY'], value['eulerZ'])
                quaternion = Quaternion(value['filterW'], value['filterX'], value['filterY'], value['filterZ'])
                DataManager().sensor_data = [SensorPart(int(key)), [gyro, acc, mag, quaternion]]
                print(DataManager().sensor_data)
                
                
            # print("------------")
        except json.JSONDecodeError as e:
            print(f"JSON 디코딩 에러: {e}")
            print(f"수신된 메시지: {message}")


    def on_error(self, ws, error):
        print(f"에러 발생: {error}")


    def on_close(self, ws, close_status_code, close_msg):
        print("웹소켓 연결이 종료되었습니다.")

