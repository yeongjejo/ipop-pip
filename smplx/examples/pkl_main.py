import pickle

# pkl 파일 경로
file_path = '/home/user/Downloads/models/smplx/SMPLX_NEUTRAL.pkl'

# pkl 파일 로드
with open(file_path, 'rb') as file:
    data = pickle.load(file, encoding='latin1')

# 데이터 출력
print(data)

print(data.keys())

print(data['joint2num'])
