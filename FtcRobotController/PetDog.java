public class PetDog extends Pet{
    String favoriteToy;

    @Override
    public void displayDetails() {
        super.displayDetails();
        System.out.println("My favorite toy is" + favoriteToy);
    }
    @Override
    public int calculateAge(){
        return this.age * 7;
    }

}
