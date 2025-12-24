#include <iostream>
#include <cstring>
using namespace std;

class Person
{
	char * name;
	int age;
public:
	Person(char * myname, int myage)
	{
		int len=strlen(myname)+1;
		name=new char[len];
		strcpy(name, myname);
		age=myage;
	}

	// 밑의 Person 복사 생성사는 깊은 복사를 위해 생성한 구문
	// 없으면 소멸자가 복사된 객체에 대해 한번만 실행 됨
	// 결과적으로 메모리 낭비가 생김
    Person(const Person &copy) : age(copy.age){
		name = new char[strlen(copy.name)+1];
		strcpy(name, copy.name);
	}

	void ShowPersonInfo() const
	{
		cout<<"이름: "<<name<<endl;
		cout<<"나이: "<<age<<endl;
	}
	~Person()
	{
		delete []name;
		cout<<"called destructor!"<<endl;
	}
};
;
int main(void)
{
	Person man1("Lee Dong Woo", 29);
	Person man2(man1);
	man1.ShowPersonInfo();
	man2.ShowPersonInfo();
	return 0;
}