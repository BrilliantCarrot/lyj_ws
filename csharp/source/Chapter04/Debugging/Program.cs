using static System.Console;

static double Add(double a, double b){
    return a * b;
}

double a = 4.5;
double b = 2.5;
double answer = Add(a, b);
WriteLine($"{a}+{b} = {answer}");
ReadLine();     // 사용자가 ENTER를 누를 때까지 대기한다.

