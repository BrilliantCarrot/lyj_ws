using Packt.Shared;
using static System.Console;

Person harry = new() { Name = "Harry" };
Person mary = new() { Name = "Mary" };
Person jill = new() { Name = "Jill" };

// Implementing functionality using methods and operators

// call instance method
Person baby1 = mary.ProcreateWith(harry);
baby1.Name = "Gary";

// call static method
Person baby2 = Person.Procreate(harry, jill);

// call an operator
Person baby3 = harry * mary;

// WriteLine($"{harry.Name} has {harry.Children.Count} children.");
// WriteLine($"{mary.Name} has {mary.Children.Count} children.");
// WriteLine($"{jill.Name} has {jill.Children.Count} children.");

// WriteLine(
//   format: "{0}'s first child is named \"{1}\".",
//   arg0: harry.Name,
//   arg1: harry.Children[0].Name);

WriteLine($"5! is {Person.Factorial(5)}");

System.Collections.Hashtable lookupObject = new();
lookupObject.Add(key: 1, value: "Apha");
lookupObject.Add(key: 2, value: "Beat");
lookupObject.Add(key: 3, value: "Gamma");
lookupObject.Add(key: harry, value: "Delta");

int key = 2;
WriteLine(format: "key {0} has value: {1}", arg0: harry, arg1: lookupObject[key]);

WriteLine(format: "key {0} has value {1}", argo: harry, arg1: lookupObject[harry]);
