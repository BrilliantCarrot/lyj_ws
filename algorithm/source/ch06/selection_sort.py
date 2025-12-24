# 단순 선택 정렬 알고리즘 구현하기

from typing import MutableSequence

def selection_sort(a:MutableSequence) -> None:
    """단순 선택 정렬"""
    n = len(a)
    for i in range(n-1):
        min = i     # 정렬할 부분에서 가장 작은 원소의 인덱스
        for j in range(i+1, n):
            if a[j]<a[min]:
                min = j
        a[i], a[min] = a[min], a[i]     # 정렬할 부분에서 맨 앞의 원소와 가장 작은 원소를 교환