public class Constructors {
    public static void main (String [] args){
        Dog myDog = new Dog("Sriman", "Poodle");
        Dog yourDog = new Dog("Srihan", "Chimpanzee");

        myDog.print();
        yourDog.print();

        myDog.setName("Vedant");
        yourDog.setName("Billy");
        myDog.setBreed("Human");
        yourDog.setBreed("Labrador");

        myDog.print();
        yourDog.print();

    }
}
