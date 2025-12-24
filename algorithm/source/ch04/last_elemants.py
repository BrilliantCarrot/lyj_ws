# 원하는 개수(n)개 만큼 값을 입력받아 마지막 n개를 저장

n = int(input("정수를 몇 개 저장할까요?"))
a = [None] * n

cnt = 0
while True:
    a[cnt % n] = int(input((f'{cnt + 1}번째 정수를 입력하세요: ')))
    cnt += 1

    retry = input(f"계속 할까요?(Y ... Yes / N ... No): ")
    if retry in {'N','n'}:
        break

i = cnt - n
if i < 0: i = 0

while i < cnt:
    print(f"{i+1}번째 = {a[i%n]}")
    i += 1