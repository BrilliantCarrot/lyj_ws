using static System.Console;
WriteLine($"There are {args.Length} argumens");

// 네 가지 인수의 값을 열거하거나 반복하려면 코드 주석을 해제
foreach(string arg in args){
    WriteLine(arg);
}