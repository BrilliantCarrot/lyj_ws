using static System.Console;
namespace Packt.Shared;
public class DvdPlayer : IPlayable{
    public void Puase(){
        WriteLine('Pause');
    }
    public void Play(){
        WriteLine("Play");
    }
}