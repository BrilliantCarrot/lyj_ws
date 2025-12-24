#include <iostream>
#include <cstring>
using namespace std;

class Person
{
private:
    char * name;
public:
    Person(const char * myname)  // const char*로 수정
    {
        name = new char[strlen(myname) + 1];
        strcpy(name, myname);
    }
    ~Person()
    {
        delete[] name;
    }
    void WhatYourName() const
    {
        cout << "My name is " << name << endl;
    }
};

class UnivStudent : public Person
{
private:
    char * major;
public:
    UnivStudent(const char * myname, const char * mymajor)  // const char*로 수정
        : Person(myname)
    {
        major = new char[strlen(mymajor) + 1];
        strcpy(major, mymajor);
    }
    ~UnivStudent()
    {
        delete[] major;
    }
    void WhoAreYou() const
    {
        WhatYourName();
        cout << "My major is " << major << endl << endl;
    }
};

int main(void)
{
    UnivStudent st1("Kim", "Mathematics");  // Mathmatics -> Mathematics 오타 수정
    st1.WhoAreYou();

    UnivStudent st2("Hong", "Physics");
    st2.WhoAreYou();
    return 0;
}