from datetime import datetime
import sys
import json
import time
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QFileDialog, QLineEdit
from PyQt5.QtCore import Qt

from data_manager import DataManager
from protocol.udp_server import UDPServer
from protocol.udp_station_broadcast_receiver import UDPStationBroadcastReceiver

class MyWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("시작 정지 버튼")
        self.resize(500, 400)  # 창 크기 설정

        # 폴더 경로와 파일 제목 저장할 변수
        self.folder_path = "/home/user/Desktop/log/ipop"
        self.file_name = datetime.now().strftime("%Y%m%d_%H%M%S.json") # 기본 파일 이름
        self.comment = "" # 기본 파일 이름

        # JSON 데이터를 저장할 변수 (예시)
        self.json_data = {}

        # 폴더 경로 텍스트 라벨
        self.path_label = QLabel("폴더 경로: /home/user/Desktop/log/ipop", self)

        # 폴더 경로 선택 버튼
        self.select_path_button = QPushButton("폴더 경로 설정", self)
        self.select_path_button.clicked.connect(self.select_path)

        # 파일 제목 입력 필드
        self.file_name_input = QLineEdit(self)
        self.file_name_input.setPlaceholderText("파일 제목을 입력하세요")
        self.file_name_input.textChanged.connect(self.update_file_name)
        
        # 코멘트 입력 필드
        self.comment_input = QLineEdit(self)
        self.comment_input.setPlaceholderText("코멘트를 입력하세요")
        self.comment_input.textChanged.connect(self.update_comment)

        # 시작 버튼
        self.start_button = QPushButton("시작", self)
        self.start_button.clicked.connect(self.toggle_button)


        # 레이아웃 설정
        layout = QVBoxLayout()
        layout.addWidget(self.path_label)
        layout.addWidget(self.select_path_button)
        layout.addWidget(self.file_name_input)
        layout.addWidget(self.comment_input)
        layout.addWidget(self.start_button)

        # 창에 레이아웃 적용
        self.setLayout(layout)

        # 엔터키로 버튼 상태 변경 가능하도록 이벤트 바인딩
        self.setFocusPolicy(Qt.StrongFocus)
        
        
        self.udp_receiver = UDPStationBroadcastReceiver()
        self.udp_server = UDPServer()

    def keyPressEvent(self, event):
        if event.key() == 16777220:  # 엔터키의 key 값
            self.toggle_button()

    def toggle_button(self):
        # 시작/정지 버튼 전환
        if self.start_button.text() == "시작":
            self.start_button.setText("시작 준비 중...")
            
            DataManager().clear()
            # self.udp_receiver.start()
            # time.sleep(1)
            # self.udp_server.start()
            print("시작!!!!!!!!!!!!!111!!!!!!!!!!!!!!!!!!!!!!")
            self.start_button.setText("정지")
        else:
            # self.udp_server.stop()
            # self.udp_receiver.stop()
            # DataManager().clear()
            self.save_json()
            self.start_button.setText("시작")

    def select_path(self):
        # 폴더 다이얼로그를 열어 폴더 경로를 선택
        options = QFileDialog.Options()
        folder_path = QFileDialog.getExistingDirectory(self, "폴더 경로 설정", "", options=options)
        if folder_path:
            self.folder_path = folder_path
            self.path_label.setText(f"폴더 경로: {self.folder_path}")

    def update_file_name(self, text):
        # 사용자가 파일 제목을 입력할 때마다 파일 이름을 업데이트
        if text:
            self.file_name = text + ".json"
        else:
            now = datetime.now()
            self.file_name = now.strftime("%Y%m%d_%H%M%S.json")
    
    def update_comment(self, text):
        if text:
            self.comment = text

    def save_json(self):
        # 폴더 경로와 파일 제목이 모두 설정되었을 때 JSON 파일 저장
        if self.folder_path:
            try:
                # JSON 파일을 폴더 경로에 저장
                file_path = f"{self.folder_path}/{self.file_name}"
                with open(file_path, 'w') as json_file:
                    q, acc = DataManager().get_log_data()
                    print(q)
                    print(acc)
                
                    self.json_data['q'] = q
                    self.json_data['acc'] = acc
                    self.json_data['comment'] = self.comment
                    print(self.json_data)
                    
                    json.dump(self.json_data, json_file, indent=4)
                print(f"파일이 성공적으로 저장되었습니다: {file_path}")
            except Exception as e:
                print(f"파일 저장 중 오류 발생: {e}")
        else:
            print("폴더 경로가 설정되지 않았습니다.")

if __name__ == "__main__":
    UDPStationBroadcastReceiver().start()
    time.sleep(1)
    UDPServer().start()
    
    app = QApplication(sys.argv)

    # 윈도우 객체 생성 및 실행
    window = MyWindow()
    window.show()

    sys.exit(app.exec_())
