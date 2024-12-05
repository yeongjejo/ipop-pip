import multiprocessing
import time

# 실행할 함수 1
def task1():
    for i in range(5):
        print(f"Task 1: {i}")
        time.sleep(1)  # 1초 대기

# 실행할 함수 2
def task2():
    for i in range(5):
        print(f"Task 2: {i}")
        time.sleep(1.5)  # 1.5초 대기

if __name__ == "__main__":
    # 프로세스 생성
    process1 = multiprocessing.Process(target=task1)
    process2 = multiprocessing.Process(target=task2)

    # 프로세스 시작
    process1.start()
    for i in range(10):
        print(i)
    process2.start()
    for i in range(10):
        print(i+50)

    # 두 프로세스가 종료될 때까지 대기
    process1.join()
    process2.join()

    print("두 프로세스가 모두 종료되었습니다.")