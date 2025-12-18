using System; // DateTime
using System.Collections.Generic; // List<T>

using static System.Console;

namespace Packt.Shared
{
  public partial class Person : object
  {
    
    // 필드를 선언
    public string Name;
    public DateTime DateOfBirth;
    public WondersOfTheAncientWorld FavoriteAncientWonder;
    public WondersOfTheAncientWorld BucketList;
    public List<Person> Children = new List<Person>();

    // 상수 필드를 선언
    public const string Species = "Homo Sapiens";

    // 읽기 전용 필드
    public readonly string HomePlanet = "Earth";
    public readonly DateTime Instantiated;

    // 생성자로 필드 초기화하기
    public Person()
    {
      // set default values for fields
      // including read-only fields 
      Name = "Unknown";
      Instantiated = DateTime.Now;
    }

    // 입력값 2개가 있는 생성자
    public Person(string initialName, string homePlanet)
    {
      Name = initialName;
      HomePlanet = homePlanet;
      Instantiated = DateTime.Now;
    }

    // deconstructors
    public void Deconstruct(out string name, out DateTime dob)
    {
      name = Name;
      dob = DateOfBirth;
    }

    public void Deconstruct(out string name,
      out DateTime dob, out WondersOfTheAncientWorld fav)
    {
      name = Name;
      dob = DateOfBirth;
      fav = FavoriteAncientWonder;
    }

    // p.300 메서드 작성 및 호출하기
    // 메서드에서 값 반환하기
    public void WriteToConsole()
    {
      WriteLine($"{Name} was born on a {DateOfBirth:dddd}.");
    }

    public string GetOrigin()
    {
      return $"{Name} was born on {HomePlanet}.";
    }

    // 튜플 구문 지원
    public (string, int) GetFruit()
    {
      return ("Apples", 5);
    }

    // 튜플 필드에 고유의 이름을 지정
    public (string Name, int Number) GetNamedFruit()
    {
      return (Name: "Apples", Number: 5);
    }

    public string SayHello()
    {
      return $"{Name} says 'Hello!'";
    }

    public string SayHello(string name)
    {
      return $"{Name} says 'Hello {name}!'";
    }

    public string OptionalParameters(
      string command = "Run!",
      double number = 0.0,
      bool active = true)
    {
      return string.Format(
        format: "command is {0}, number is {1}, active is {2}",
        arg0: command, 
        arg1: number, 
        arg2: active);
    }

    // 매개 변수 전달 제에허기
    public void PassingParameters(int x, ref int y, out int z)
    {
      // out parameters cannot have a default
      // AND must be initialized inside the method 
      z = 99;

      // increment each parameter 
      x++;
      y++;
      z++;
    }
  }
}
