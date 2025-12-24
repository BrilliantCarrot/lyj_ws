using System;
using System.Collections;

class MainClass
{
    static void Main(string[] args)
    {
        // int[] array1 = new int[3];
        // array1[0] = 10;
        // array1[1] = 20;
        // array1[2] = 30;


        // int[] array2 = new int[] { 1, 2, 3 };



        // int[] array3 = { 4, 5, 6 };

        // Console.WriteLine(array1);
        // Console.WriteLine(array2[0]);



        // ArrayList 학습
        // ArrayList는 다양한 타입의 데이터를 저장할 수 있는 동적 배열입니다.
        // ArrayList는 System.Collections 네임스페이스에 포함되어 있습니다.
        // ArrayList는 타입 안정성이 없기 때문에, 다양한 타입의 데이터를 저장할 수 있지만,
        // 런타임 시 타입 오류가 발생할 수 있습니다.


        // ArrayList al = new ArrayList();
        // al.Add(1);
        // al.Add("Hello");
        // al.Add(3.3);
        // al.Add(true);

        // foreach (var item in al)
        // {
        //     Console.WriteLine(item);
        // }
        // Console.WriteLine();

        // al.Remove("Hello");

        // foreach(var item in al)
        // {
        //     Console.WriteLine(item);
        // }

        Hashtable ht = new Hashtable();
        ht.Add("name", "John");
        ht.Add("age", 30);
        ht.Add("isStudent", false);

        Console.WriteLine("HashTable contents:");
        foreach (DictionaryEntry entry in ht)
        {
            Console.WriteLine($"{entry.Key}: {entry.Value}");
        }       
    }

}