import os

# 폴더 경로를 설정하세요
folder_path = 'C:/Users/leeyj/Downloads/imageFileEdit'

# 폴더 내 모든 파일을 순회합니다
for filename in os.listdir(folder_path):
    # 파일의 현재 경로
    current_file_path = os.path.join(folder_path, filename)

    # 파일 이름 끝에 '.png' 추가
    new_filename = filename + '.png'
    new_file_path = os.path.join(folder_path, new_filename)

    # 파일 이름을 변경
    os.rename(current_file_path, new_file_path)

print("모든 파일에 .png가 추가되었습니다.")
