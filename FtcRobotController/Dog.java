public class Dog {
    String name;
    String breed;
    public Dog (String name, String breed){
        this.name = name;
        this.breed = breed;
    }
    public Dog (String name){
        this.name = name;
    }

    public void print() {
        System.out.println(this.name + " is a " + this.breed);
    }
    public void setBreed(String breed){
        this.breed = breed;
    }
    public void setName(String name) {
        this.name = name;
    }
}

