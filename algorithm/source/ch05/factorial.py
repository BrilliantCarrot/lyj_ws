def factorial(n: int) -> int:
    if n>0:
        return n * factorial(n-1)
    else:
        return 1
    
if __name__ == "__main__":
    n = int(input("출력할 팩토리얼 값을 입력하세요: "))
    print(f"{n}의 팩토리얼 값은 {factorial(n)}입니다.")