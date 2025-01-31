public class PetBird extends Pet{
    String wingSpan;

    @Override
    public void displayDetails() {
        super.displayDetails();
        System.out.println("My wingspan is" + wingSpan);
    }
    @Override
    public int calculateAge(){
        return this.age * 9;
    }
}
