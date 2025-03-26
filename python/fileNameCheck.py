import os
import re

def find_missing_numbers(folder_path):
    # 파일 이름에서 숫자를 추출하는 정규 표현식 패턴
    pattern = re.compile(r'(\d+)')
    
    # 폴더 내 파일의 숫자 추출
    numbers = []
    for filename in os.listdir(folder_path):
        match = pattern.search(filename)
        if match:
            numbers.append(int(match.group()))
    
    # 파일이 없거나 숫자를 추출할 수 없는 경우
    if not numbers:
        print("폴더에 숫자 패턴을 포함한 파일이 없습니다.")
        return

    # 정렬된 숫자 목록
    numbers.sort()
    
    # 빠진 숫자 찾기
    missing_numbers = [num for num in range(numbers[0], numbers[-1] + 1) if num not in numbers]
    
    if missing_numbers:
        print("빠진 숫자:", missing_numbers)
    else:
        print("모든 숫자가 연속적입니다.")

# 사용 예시
folder_path = 'C:/Users/leeyj/Downloads/3115155_c01c26c935'
find_missing_numbers(folder_path)