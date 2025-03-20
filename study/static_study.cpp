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


  // typedef struct Animal {
  //   char name[30];  // 이름
  //   int age;        // 나이
  
  //   int health;  // 체력
  //   int food;    // 배부른 정도
  //   int clean;   // 깨끗한 정도
  // } Animal;
  
  // void create_animal(Animal *animal) {
  //   std::cout << "동물의 이름? ";
  //   std::cin >> animal->name;
  
  //   std::cout << "동물의 나이? ";
  //   std::cin >> animal->age;
  
  //   animal->health = 100;
  //   animal->food = 100;
  //   animal->clean = 100;
  // }
  
  // void play(Animal *animal) {
  //   animal->health += 10;
  //   animal->food -= 20;
  //   animal->clean -= 30;
  // }
  // void one_day_pass(Animal *animal) {
  //   // 하루가 지나면
  //   animal->health -= 10;
  //   animal->food -= 30;
  //   animal->clean -= 20;
  // }
  // void show_stat(Animal *animal) {
  //   std::cout << animal->name << "의 상태" << std::endl;
  //   std::cout << "체력    : " << animal->health << std::endl;
  //   std::cout << "배부름 : " << animal->food << std::endl;
  //   std::cout << "청결    : " << animal->clean << std::endl;
  // }


  // Animal 클래스
  class Animal{
    private:
      int food;
      int weight;

    public:
    void set_animal(int _food, int _weight){
      food = _food;
      weight = _weight;
    }
    void increase_food(int inc){
      food += inc;
      weight += (inc / 3);
    }
    void view_stat(){
      std::cout << "이 동물의 food  : " << food << std::endl;
      std::cout << "이 동물의 weight  : " << weight << std::endl;
    }
  }; // 세미콜론 잊지 말자


  // 달력 클래스
  class Date {
    int year_;
    int month_;  // 1 부터 12 까지.
    int day_;    // 1 부터 31 까지.
  
   public:
    void SetDate(int year, int month, int date){
      // 날짜가 한 달 안에 포함되는지를 확인하는 함수
      year_ = year;
      month_ = month;
      day_ = date;
    };
    void AddDay(int inc){
      //더하려는 날짜가 한 달 안에 포함되있는지를 확인하는 함수
    };
    void AddMonth(int inc);
      // 12월을 넘는지를 확인하는 계산 
    void AddYear(int inc){
      year_ += inc;
    };
    
    void ShowDate(){
      std::cout << year_ << "년" << month_ << "월" << day_ << "일입니다." << std::endl;
    };
  };

int main(){
    //   
  
  
  
    // 날짜 과제
    // Date date;
    // int year, month, day;
    // std::cout << "날짜 프로그램" << std::endl;
    
    // while(true){
    //   std::cout << "연도 입력: " << std::endl;
    //   std::cin >> year;
    //   date.AddYear(year);
    //   std::cout << "월 입력: " << std::endl;
    //   std::cin >> month;
    //   // 월 입력 처리 부분(날짜 확인 포함함)
    //   date.AddMonth(month);
    //   std::cout << "날짜 입력: " << std::endl;
    //   std::cin >> day;
    //   date.AddDay(day);
    // }



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
    

    // int *p = new int;
    // *p = 10;
    // std::cout << *p << std::endl;
    // delete p;


    // int arr_size;
    // std::cout << "array size : ";
    // std::cin >> arr_size;
    // int *list = new int[arr_size];
    // for (int i = 0; i < arr_size; i++) {
    //   std::cin >> list[i];
    // }
    // for (int i = 0; i < arr_size; i++) {
    //   std::cout << i << "th element of list : " << list[i] << std::endl;
    // }
    // delete[] list;


    // animal 구조체
    // Animal *list[10];
    // int animal_num = 0;
  
    // for (;;) {
    //   std::cout << "1. 동물 추가하기" << std::endl;
    //   std::cout << "2. 놀기 " << std::endl;
    //   std::cout << "3. 상태 보기 " << std::endl;
  
    //   int input;
    //   std::cin >> input;
  
    //   switch (input) {
    //     int play_with;
    //     case 1:
    //       list[animal_num] = new Animal;
    //       create_animal(list[animal_num]);
  
    //       animal_num++;
    //       break;
    //     case 2:
    //       std::cout << "누구랑 놀게? : ";
    //       std::cin >> play_with;
  
    //       if (play_with < animal_num) play(list[play_with]);
  
    //       break;
  
    //     case 3:
    //       std::cout << "누구껄 보게? : ";
    //       std::cin >> play_with;
    //       if (play_with < animal_num) show_stat(list[play_with]);
    //       break;
    //   }
  
    //   for (int i = 0; i != animal_num; i++) {
    //     one_day_pass(list[i]);
    //   }
    // }
    // for (int i = 0; i != animal_num; i++) {
    //   delete list[i];
    // }

    // Animal animal;
    // animal.set_animal(100, 50);
    // animal.increase_food(30);

    // animal.view_stat();


    // return 0;
}

