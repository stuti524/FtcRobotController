public class Pet {
    String name;
    int age;
    String species;

    public void displayDetails(){
        System.out.println(this.name + " is " + this.age + " years old and is a " + species);
    }
    public int calculateAge(){
        return this.age;
    }
}

