#include <iostream>
using namespace std;

class SoSimple{
    private:
        static int simObjCnt;
    public:
        SoSimple(){
            simObjCnt++;
            cout << simObjCnt << " 번째 SoSimple 객체" << endl;
        }
};
int SoSimple::simObjCnt = 0;

class SoComplex{
    private:
        static int cmxObjCnt;
    public:
    SoComplex(){
        cmxObjCnt++;
        cout << cmxObjCnt << " 번째 SoComplex 객체" << endl;
    }
    SoComplex(SoComplex & copy){
        cmxObjCnt++;
        cout << cmxObjCnt << " 번째 SoComplex 객체" << endl;
    }
};
int SoComplex::cmxObjCnt = 0;

int main(void)
{
	SoSimple sim1;
	SoSimple sim2;

	SoComplex cmx1;
	SoComplex cmx2=cmx1;    // 복사 생성자
	SoComplex();            // 임시 객체
	return 0;
}