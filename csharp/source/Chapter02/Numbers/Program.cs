using System.Xml;
using static System.Console;

object height = 1.88; // storing a double in an object
object name = "Amir"; // storing a string in an object
// Console.WriteLine($"{name} is {height} metres tall.");

// int length1 = name.Length; // gives compile error!
int length2 = ((string)name).Length; // tell compiler it is a string
// Console.WriteLine($"{name} has {length2} characters.");

// storing a string in a dynamic object
// string has a Length property
dynamic something = "Ahmed";

// int does not have a Length property
// something = 12;

// an array of any type has a Length property
// something = new[] { 3, 5, 7 };

// this compiles but would throw an exception at run-time
// if you had stored a value with a data type that does not 
// have a property named Length
int length = something.Length;

var population = 66_000_000; // 66 million in UK 
var weight = 1.88; // in kilograms
var price = 4.99M; // in pounds sterling
var fruit = "Apples"; // strings use double-quotes 
var letter = 'Z'; // chars use single-quotes
var happy = true; // Booleans have value of true or false

// good use of var because it avoids the repeated type
// as shown in the more verbose second statement
var xml1 = new XmlDocument();
XmlDocument xml2 = new XmlDocument();

// bad use of var because we cannot tell the type, so we
// should use a specific type declaration as shown in
// the second statement
var file1 = File.CreateText("something1.txt");
StreamWriter file2 = File.CreateText("something2.txt");

XmlDocument xml3 = new(); // target-typed new in C# 9 or later

// class Person is defined at the bottom of the file
Person kim = new();
kim.BirthDate = new(1967, 12, 26); // instead of: new DateTime(1967, 12, 26)


WriteLine($"default(int) = {default(int)}");
// WriteLine($"default(bool) = {default(bool)}");
// WriteLine($"default(DateTime) = {default(DateTime)}");
// WriteLine($"default(string) = {default(string)}");

// int number = 13;
// WriteLine($"number has been set to: {number}");
// number = default;
// WriteLine($"number has been reset to its default: {number}");

// 부호없는 정수(unsigned integer)는 양의 정수 또는 0을 의미한다
uint naturalNumber = 23;
int integerNumber = -23;
// F 접미사를 사용하면 flot 리터럴이 된다
float realNumber = 2.3F;
double anotherRealNumber = 2.3;

// 숫자 200마나을 저장하는 3개의 변수
int decimalNotation = 2_000_000;
int binaryNotation = 0b_0001_1110_1000_0100_1000_0000;
int hexadecimalNotation = 0X_001E_8480;


// string[] names = new string[4];
// names[0] = "Kate";
// names[1] = "Jack";
// names[2] = "Rebecca";
// names[3] = "Tom";
// for (int i = 0; i< names.Length; i++){
//   WriteLine(names[i]);
// }


int numberOfApples = 12;
decimal pricePerApple = 0.35M;

// 문자열 형식 지정에 익숙해지면 format:, arg0:, arg1: 같은 매개 변수 이름 지정은 사용하지 않는다
// WriteLine(
//     format: "{0} apples costs {1:C}",
//     arg0: numberOfApples,
//     arg1: pricePerApple * numberOfApples);

// string formatted = string.Format(
//     format: "{0} apples costs {1:C}",
//     arg0: numberOfApples,
//     arg1: pricePerApple * numberOfApples);

// 문자열 보간
// 접두사 $로 시작하는 문자열 안에 string 변수를 중괄호로 감싸서 입력하면 string 변수의 현재 값이 입력 위치에 출력된다
// 피해서 사용하는것이 좋다
// WriteLine($"{numberOfApples} apples costs {pricePerApple * numberOfApples}");

// WriteLine($"int uses {sizeof(int)} bytes and can store numbers in the range {int.MinValue:N0} to {int.MaxValue:N0}.");
// WriteLine($"double uses {sizeof(double)} bytes and can store numbers in the range {double.MinValue:N0} to {double.MaxValue:N0}.");
// WriteLine($"decimal uses {sizeof(decimal)} bytes and can store numbers in the range {decimal.MinValue:N0} to {decimal.MaxValue:N0}.");

// 사용자의 입력을 받고 출력
// Write("Type your first name and press ENTER: ");
// string? firstName = ReadLine();
// Write("Type your age and press ENTER: ");
// string? age = ReadLine();
// WriteLine($"Hello {firstName}, you look good for {age}");

// 사용자에게 키 입력받기
Write("Press any key combination: ");
ConsoleKeyInfo key = ReadKey();
WriteLine();
WriteLine("key: {0}, char: {1}, Modifiers: {2}", arg0: key.Key, arg1: key.KeyChar, arg2: key.Modifiers);

class Person
{
  public DateTime BirthDate;
}