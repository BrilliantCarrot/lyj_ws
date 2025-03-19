#include <iostream>


// static 멤버 변수
// void counter() {
//     static int count = 0;  // 최초 호출 시 한 번만 초기화됨
//     count++;               // 값이 유지됨
//     std::cout << "count: " << count << std::endl;
// }

// int main() {
//     counter(); // 출력: count: 1
//     counter(); // 출력: count: 2
//     counter(); // 출력: count: 3
//     return 0;
// }


// static 멤버 함수
// #include <iostream>

// class Counter {
// private:
//     static int count;
// public:
//     static void increment() { count++; } // 정적 멤버 함수
//     static int getCount() { return count; }
// };

// int Counter::count = 0; // 정적 변수 초기화

// int main() {
//     Counter::increment(); // 객체 없이 호출 가능
//     Counter::increment();
//     std::cout << "Count: " << Counter::getCount() << std::endl; // 출력: 2
// }
// Counter::increment()처럼 객체 없이 호출 가능.
// 일반 멤버 변수에는 접근할 수 없고, 정적 멤버 변수만 조작 가능.


// 파일에서 static 사용
// file1.cpp
// #include <iostream>

// static int hidden = 10;  // 이 파일에서만 사용 가능

// void show() {
//     std::cout << "Hidden: " << hidden << std::endl;
// }

// int main() {
//     show();
//     return 0;
// }

// 함수 인자로 레퍼런스 받기
int change_val(int & p){
    p = 3;
}
// const를 이용한 예외 규칙
int function() {
    int a = 5;
    return a;
    }


int main(){
    // std::cout << "hi" << std::endl
    // << "my name is "
    // << "Psi" << std::endl;
    

    // int number = 5;
    // std::cout << number << std::endl;
    // change_val(number);
    // std::cout << number << std::endl;


    // 배열 형태로 사용
    // int arr[3] = {1, 2, 3};
    // int(&ref)[3] = arr;
    // ref[0] = 2;
    // ref[0] = 3;
    // ref[0] = 1;
    // std::cout << arr[0] << arr[1] << arr[2] << std::endl;


    // const int& c = function();
    // std::cout << "c : " << c << std::endl;
    

    int *p = new int;
    *p = 10;
    std::cout << *p << std::endl;
    delete p;






    return 0;
}

