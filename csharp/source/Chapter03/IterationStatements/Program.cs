using static System.Console;

// int x = 0;
// while (x < 10){
//     WriteLine(x);
//     x++;
// }

// string? password;
// do{
//     Write("Enter your password: ");
//     password = ReadLine();
// }
// while(password != "Pa$$w0rd");
// WriteLine("Correct!");

string[] names = {"Adam","Barry","Charlie"};
foreach(string name in names){
    WriteLine($"{name} has {name.Length} characters");
}

// 배열이나 컬렉션처럼 여러 항목을 갖는 형식을 만드는경우 
// foreach로 해당 형식의 항목을 열거할 수 있는지를 확인해야 한다

// IEnumerator e = names.GetEnumerator();
// while(e.MoveNext()){
//     string name = (string)e.Current;    // Current 속성은 읽기 전용이다
//     WriteLine($"{name}has {name.Length}characters.");
// }

// Looping with the do statement
// 
// string? actualPassword = "Pa$$w0rd";
// string? password;
// int maximumAttempts = 10;
// int attempts = 0;

// do
// {
//   attempts++;
//   Write("Enter your password: ");
//   password = ReadLine();
// }
// while ((password != actualPassword) & (attempts < maximumAttempts));

// if (password == actualPassword)
// {
//   WriteLine("Correct!");
// }
// else
// {
//   WriteLine("You have used {0} attempts! The password was {1}.",
//     arg0: maximumAttempts, arg1: actualPassword);
// }

// // Looping with the for statement

// for (int y = 1; y <= 10; y++)
// {
//   WriteLine(y);
// }

// // Looping with the foreach statement

// string[] names = { "Adam", "Barry", "Charlie" };

// foreach (string name in names)
// {
//   WriteLine($"{name} has {name.Length} characters.");
// }
