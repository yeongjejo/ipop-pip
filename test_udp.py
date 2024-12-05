import os
import socket
import win32api

def udp_server():
    is_executable = False
    server_for_unity = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_for_unity.bind(('192.168.201.201', 50001))  # 서버를 192.168.201.201 IP에 바인딩
    server_for_unity.listen(1)
    print('Server start. Waiting for unity3d to connect.')

    if '' != '' and os.path.exists(''):
        win32api.ShellExecute(0, 'open', os.path.abspath(''), '', '', 1)
        is_executable = True

    # Unity에서 연결 대기
    conn, addr = server_for_unity.accept()
    print(f"Unity 연결 성공: {addr}")  # addr 출력 확인

    # 서버의 UDP 포트 설정
    UDP_IP = "0.0.0.0"  # 모든 IP 주소에서 요청을 받음
    UDP_PORT = 5005  # 사용할 포트 번호

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 소켓을 IP와 포트에 바인딩
    sock.bind((UDP_IP, UDP_PORT))
    print(f"UDP 서버가 {UDP_IP}:{UDP_PORT}에서 대기 중입니다...")

    i = 0
    # 무한 루프를 통해 서버가 계속 동작하도록 설정
    while True:
        # 데이터를 수신 (최대 1317바이트)
        data, addr = sock.recvfrom(1317)
        print(f"{i} 클라이언트 {addr}로부터 수신된 메시지: {data.decode()}")
        i += 1

        # 수신된 데이터를 TCP 클라이언트(conn)에게 전송
        try:
            conn.send(data.decode().encode('utf8'))
            print("TCP 클라이언트로 데이터 전송 완료")
        except Exception as e:
            print(f"데이터 전송 중 오류 발생: {e}")
            break


udp_server()