# 8퀸 문제 알고리즘 구현하기

pos = [0] * 8           # 각 열에서 퀸의 위치
flag_a = [False] * 8    # 각 행에 퀸을 배치했는지 체크
flag_b = [False] * 15  # 대각선 방향(↙↗)으로 퀸을 배치했는지 체크
flag_c = [False] * 15  # 대각선 방향( ↘↖)으로 퀸을 배치했는지 체크

def put() -> None:
    # """각 열에 배치한 퀸의 위치를 출력"""
    # for i in range(8):
    #     print(f"{pos[i]:2}", end="")
    # print()
    """퀸을 놓는 상황을 □와 ■로 출력"""
    for j in range(8):
        for i in range(8):
            print('■' if pos[i] == j else '□', end='')
        print()
    print()

def set(i:int) -> None:
    """i열에 퀸을 배치"""
    for j in range(8):
        if(     not flag_a[j]               # 퀸을j행에 배치하지 않았으면
            and not flag_b[i+j]             # 대각선 방향(↙↗)으로 퀸을 배치했는지 체크
            and not flag_c[i-j+7]):         # 대각선 방향( ↘↖)으로 퀸을 배치했는지 체크
            pos[i] = j                      # 퀸을 j행에 배치
            if i == 7:                      # 모든 열에 퀸 배치를 완료
                put()
            else:
                flag_a[j] = flag_b[i + j] = flag_c[i - j + 7] = True
                set(i + 1)  # 다음 열에 퀸을 배치
                flag_a[j] = flag_b[i + j] = flag_c[i - j + 7] = False

set(0)