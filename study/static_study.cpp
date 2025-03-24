#include <iostream>
# include <string.h>

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
    int month_;  // 1부터 12까지
    int day_;    // 1부터 31까지

    // 해당 연도의 윤년 여부 판단 (윤년이면 2월은 29일까지)
    bool IsLeapYear(int year) {
        return (year % 400 == 0) || ((year % 4 == 0) && (year % 100 != 0));
    }

    // 주어진 연도와 월에 맞는 일수를 반환
    int DaysInMonth(int year, int month) {
        switch(month) {
            case 2: return IsLeapYear(year) ? 29 : 28;
            case 4: case 6: case 9: case 11: return 30;
            default: return 31;
        }
    }

public:
    // 디폴트 생성자: 기본 날짜를 2000년 1월 1일로 설정
    Date() : year_(2000), month_(1), day_(1) { }

    Date(int year, int month, int day){
      std::cout << "인자 3개인 생성자 호출" << std::endl;
      year_ = year;
      month_ = month;
      day_ = day;
    }

    // 날짜 초기화
    void SetDate(int year, int month, int day) {
        year_ = year;
        month_ = month;
        day_ = day;
    }

    // 일 수 만큼 더하는 함수
    void AddDay(int inc) {
        day_ += inc;
        // day_가 현재 월의 일수를 초과하는 경우 다음 달로 넘김
        while(day_ > DaysInMonth(year_, month_)) {
            day_ -= DaysInMonth(year_, month_);
            AddMonth(1);
        }
        // 만약 음수가 되는 경우 (여기서는 주로 양수 증가를 가정)
        while(day_ <= 0) {
            AddMonth(-1);
            day_ += DaysInMonth(year_, month_);
        }
    }

    // 월 수 만큼 더하는 함수
    void AddMonth(int inc) {
        int totalMonths = month_ + inc;
        if(totalMonths > 0) {
            // 총 개월수를 12로 나눈 몫은 연도에, 나머지는 월에 반영
            year_ += (totalMonths - 1) / 12;
            month_ = (totalMonths - 1) % 12 + 1;
        } else {
            // 음수 혹은 0일 때: (예: 1월에서 -1월 하면 12월 전년으로)
            int n = (-totalMonths) / 12 + 1;
            year_ -= n;
            totalMonths += n * 12;
            month_ = totalMonths;
        }
        // 현재 일(day_)이 새 월의 최대 일수보다 크면 조정
        int dim = DaysInMonth(year_, month_);
        if(day_ > dim)
            day_ = dim;
    }

    // 연 수 만큼 더하는 함수
    void AddYear(int inc) {
        year_ += inc;
        // 2월 29일인 경우, 윤년이 아니면 28일로 조정
        if(month_ == 2 && day_ == 29 && !IsLeapYear(year_))
            day_ = 28;
    }

    // 함수 선언만 객체 내부에 하고 정의는 밖에 해도됨
    void ShowDate();
};

    // 날짜 출력 함수, 정의를 밖에 함
    void Date::ShowDate() {
      std::cout << year_ << "년 " << month_ << "월 " << day_ << "일" << std::endl;
  }


  // class Point{
  //   int x, y;

  //   public:
  //     Point(int pos_x, int pos_y);
  // };

  // class Geometry{
  //   private:
  //     int num_points;
  //     Point *point_array[100];

  //   public:

  //     Geometry() : num_points(0) {}
  //     void AddPoint(const Point & point){
  //       point_array[num_points++] = new Point(point.x, point.y);
  //     }
  //     // 모든 점들 간의 거리를 출력하는 함수
  //     void PrintDistance(){

  //     }
  //     // 모든 점들을 잇는 직선들 간의 교점의 수를 출력해주는 함수 입니다.
  //     // 참고적으로 임의의 두 점을 잇는 직선의 방정식을 f(x,y) = ax+by+c = 0
  //     // 이라고 할 때 임의의 다른 두 점 (x1, y1) 과 (x2, y2) 가 f(x,y)=0 을 기준으로
  //     // 서로 다른 부분에 있을 조건은 f(x1, y1) * f(x2, y2) <= 0 이면 됩니다.
  //     void PrintNumMeets(){

  //     }
  // };


  // 스타크래프트
  class Marine{
    private:
      int hp;
      int coord_x, coord_y;
      int damage;
      bool is_dead;
      char *name;

    public:
      Marine();
      Marine(int x, int y);
      Marine(int x, int y, const char *marine_name);

      int attack();
      void be_attacked(int damage_earn);
      void move(int x, int y);

      void show_status();

  };
  Marine::Marine(){
    hp = 50;
    coord_x, coord_y = 0;
    damage = 5;
    is_dead = false;
  }
  Marine::Marine(int x, int y) {
    coord_x = x;
    coord_y = y;
    hp = 50;
    damage = 5;
    is_dead = false;
    name = NULL;
  }
  Marine::Marine(int x, int y, const char* marine_name) {
    name = new char[strlen(marine_name) + 1];
    strcpy(name, marine_name);
  
    coord_x = x;
    coord_y = y;
    hp = 50;
    damage = 5;
    is_dead = false;
  }
  void Marine::move(int x, int y) {
    coord_x = x;
    coord_y = y;
  }
  int Marine::attack() { return damage; }
  void Marine::be_attacked(int damage_earn) {
    hp -= damage_earn;
    if (hp <= 0) is_dead = true;
  }
  void Marine::show_status() {
    std::cout << " *** Marine : " << name << " ***" << std::endl;
    std::cout << " Location : ( " << coord_x << " , " << coord_y << " ) "
              << std::endl;
    std::cout << " HP : " << hp << std::endl;
  }

  int main()
  {
    // 스타크래프트
    // Marine marine1(2, 3);
    // Marine marine2(3, 5);
    // marine1.show_status();
    // marine2.show_status();
    // std::cout << std::endl << "마린 1 이 마린 2 를 공격! " << std::endl;
    // marine2.be_attacked(marine1.attack());
    // marine1.show_status();
    // marine2.show_status();

    Marine* marines[100];
    marines[0] = new Marine(2, 3, "Marine 2");
    marines[1] = new Marine(1, 5, "Marine 1");
    marines[0]->show_status();
    marines[1]->show_status();
    std::cout << std::endl << "마린 1 이 마린 2 를 공격! " << std::endl;
    marines[0]->be_attacked(marines[1]->attack());
    marines[0]->show_status();
    marines[1]->show_status();
    delete marines[0];
    delete marines[1];
  
    // 날짜 과제
    // Date d;
    // int year, month, day;
    // std::cout << "날짜 프로그램" << std::endl;
    // while(true){
    //   std::cout << "연도 입력: " << std::endl;
    //   std::cin >> year;
    //   d.AddYear(year);
    //   std::cout << "월 입력: " << std::endl;
    //   std::cin >> month;
    //   // 월 입력 처리 부분(날짜 확인 포함함)
    //   d.AddMonth(month);
    //   std::cout << "날짜 입력: " << std::endl;
    //   std::cin >> day;
    //   d.AddDay(day);
    // Date day2(2012, 10, 31);
    // day2.ShowDate();


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

    return 0;
}

