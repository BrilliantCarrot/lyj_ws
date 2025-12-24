using System;

class Cat
{
    public string Name = null;
    public int Weight = 0;

    public Cat(string name)
    {
        Name = name;
        Console.WriteLine("Cat's name is: " + Name);


    }
    public Cat(string name, int weight)
    {
        Name = name;
        Weight = weight;
        Console.WriteLine("Cat's name is: " + Name + ", weight is: " + weight);


    }
    ~Cat()
    {
        Console.WriteLine("Cat's name is: " + Name + " is being destroyed");
    }
}
class MainClass
{
    static void Main(string[] args)
    {
        Cat coco = new Cat("Coco");
        Cat moly = new Cat("Moly",10);
       
    }
}