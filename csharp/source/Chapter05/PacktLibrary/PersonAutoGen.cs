namespace Packt.Shared
{
  public partial class Person
  {
    // a property defined using C# 1 - 5 syntax 
    public string Origin
    {
      get
      {
        return $"{Name} was born on {HomePlanet}";
      }
    }

    // two properties defined using C# 6+ lambda expression body syntax
    public string Greeting => $"{Name} says 'Hello!'";

    public int Age => System.DateTime.Today.Year - DateOfBirth.Year;

    // 설정 가능한 속성을 만들려면 이전 구문을 사용해야 하며 get - set 메서드 쌍을 제공해야 한다
    public string FavoriteIceCream { get; set; } // auto-syntax

    private string favoritePrimaryColor;
    public string FavoritePrimaryColor
    {
      get
      {
        return favoritePrimaryColor;
      }
      set
      {
        switch (value.ToLower())
        {
          case "red":
          case "green":
          case "blue":
            favoritePrimaryColor = value;
            break;
          default:
            throw new System.ArgumentException(
              $"{value} is not a primary color. " +
              "Choose from: red, green, blue.");
        }
      }
    }

    // indexers
    public Person this[int index]
    {
      get
      {
        return Children[index];
      }
      set
      {
        Children[index] = value;
      }
    }
  }
}