import java.util.ArrayList; // import the ArrayList class
import java.util.Arrays;
import java.util.Random;

public class MusicLibrary {
    ArrayList<String> songs = new ArrayList<String>(Arrays.asList("Believer", "Die With a Smile", "Thunder"));

    Random random = new Random();

    public void addSong(String Song){
        songs.add(Song);
    }
    public void randomSong(String Song){
        int randomIndex = random.nextInt(songs.size());
        String randomElement = songs.get(randomIndex);
        System.out.println(randomElement);
    }
    public void removeSong(String Song){
        songs.remove(Song);
    }
    public void printList(){
        System.out.println(songs);
    }


}
