import threading
import socket

class UDPStationBroadcastReceiver(threading.Thread):
    def __init__(self, port):
        super().__init__()
        self._running = True
        self.port = port

    def stop(self):
        self._running = False  # 스레드 종료 플래그 설정
        self.join()  # 스레드가 종료될 때까지 대기

    def run(self):

        self._running = True
        self.broadcast_receiver()

    def broadcast_receiver(self):
        try:
            port = 65000
            ds = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            ds.bind(('', port))  # 모든 인터페이스에 바인딩

            udp_client_send_check = False

            while self._running:
                buffer = bytearray(11)  # 수신할 데이터 사이즈 설정

                # print(11111111111111111111)
                data, addr = ds.recvfrom(len(buffer))  # 데이터 수신
                # print(22222222222222)
                
                # 클라이언트 IP 주소 및 포트 번호 확인
                client_ip = addr[0]
                client_port = addr[1]

                # 아이피 주소 저장
                ip_num = f"{data[2]}.{data[3]}.{data[4]}.{data[5]}"

                # 시리얼 번호 저장
                serial = (data[6] << 8) | data[7]  # byte 값을 int로 변환 후 결합
                # print(serial)
                # print(f"IP 번호: {ip_num}, 시리얼: {serial}, Port6: {port6}, Port7: {port7}, 채널: {ch}")

                # 포트 번호 (분할) 저장 (880 포트라고 가정)
                # port_num = 56775  # 736포트
                port_num = self.port  # 736포트
                port6 = (port_num >> 8) & 0xFF
                port7 = port_num & 0xFF

                # 채널 정보 저장
                ch = data[8]


                print(f"IP 번호: {ip_num}, 시리얼: {serial}, Port6: {port6}, Port7: {port7}, 채널: {ch}")

                # 클라이언트에 대한 추가 작업 수행
                if not udp_client_send_check:
                    self.udp_client(ip_num, port6, port7)  # 필요한 작업 수행
                    udp_client_send_check = True

            ds.close()
        except Exception as e:
            print(f"Error1: {e}")

    def udp_client(self, ip, port6, port7):
        try:
            # UDP 소켓 생성
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            # 송신할 데이터 설정
            send_data = bytearray(10)
            send_data[0] = 0xFA
            send_data[1] = 0xEA
            send_data[2] = (192) & 0xFF  # IP 주소
            send_data[3] = (168) & 0xFF
            send_data[4] = (201) & 0xFF
            send_data[5] = (104) & 0xFF
            send_data[6] = port6  # 포트 번호
            send_data[7] = port7
            send_data[8] = 0xFB
            send_data[9] = 0xFF

            # IP 주소와 포트 설정
            server_address = (ip, 65000)
            # 데이터 전송
            sock.sendto(send_data, server_address)

        except Exception as e:
            print(f"Error2: {e}")
