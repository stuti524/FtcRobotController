package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


//LEFT JOYSTICK - FORWARD and BACKWARD
// RIGHT JOYSTICK - RIGHT and LEFT Turns


@Disabled
@Autonomous(name = "AutoDrive", group = "Auto")
public class Auto extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftFrontDrive   = null;
    public DcMotor leftBackDrive   = null;
    public DcMotor  rightFrontDrive  = null;
    public DcMotor  rightBackDrive  = null;
    public DcMotor verticalLeft = null;
    public DcMotor verticalRight = null;

    enum VertE {
        ABOVE_CHAM,
        ON_CHAM,
        RESET_CHAM
    };
    VertE state = VertE.ABOVE_CHAM;
    enum ClawServoE {
        //INTAKE_NO_DOWN_UP,
        CLAW_CLOSE,
        CLAW_OPEN
    };

    Servo clawservo;
    ClawServoE clawservoe1= ClawServoE.CLAW_CLOSE;
    VertE vertchame= VertE.RESET_CHAM;
    public boolean extended_vert = false;

    @Override
    public void runOpMode() {
        //Variables for Drive
        double left=0;
        double right=0;
        double drive=0;
        double turn=0;
        double max=0;



        // Define and Initialize Motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "cm2");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "cm3");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "cm0");
        rightBackDrive = hardwareMap.get(DcMotor.class, "cm1");
        verticalLeft = hardwareMap.get(DcMotor.class, "em1");
        verticalRight = hardwareMap.get(DcMotor.class, "em2");

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawservo = hardwareMap.get(Servo.class, "cs5");

        Servo servostop1;
        Servo servostop2;

        servostop1 = hardwareMap.get(Servo.class, "cs3");
        servostop2 = hardwareMap.get(Servo.class, "cs1");



        //Wall

        verticalLeft.setTargetPosition(0);
        verticalRight.setTargetPosition(0);

        verticalLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        verticalLeft.setPower(0.5);
        verticalRight.setPower(0.5);
        //rotateServo = hardwareMap.get(Servo.class, "es3");

        //SPIN SERVO
        //spinServo = hardwareMap.get(CRServo.class, "es5");
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setTargetPosition(0);
        leftBackDrive.setTargetPosition(0);
        rightFrontDrive.setTargetPosition(0);
        rightBackDrive.setTargetPosition(0);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);



        waitForStart();


//        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
//            leftFrontDrive.setPower(0.4);
//            leftBackDrive.setPower(0.4);
//            rightFrontDrive.setPower(0.4);
//            rightBackDrive.setPower(0.4);
//        }
        boolean extended = false;
        boolean attach = false;

        if(opModeIsActive()){

            servostop1.setPosition(0.95);
            servostop2.setPosition(0.35);
            //telemetry();

            //Park Code
            //runDistance(37.75);

            // AUTO CODE WITH NO ARCS - VALUES NOT NOT TESTED; INTAKE IS FRONT
            runDistance(-24.0, true, 0.4);
            hangSpecimen(1);   //hanging specimen 1 on the chamber
            runDistance(19, true, 0.5); //move forward (positive y distance is approx)
            turn(-90.0);                             //turn 90 degrees to the left
            runDistance(-45.5, true, 0.4);    //move backward (negative z distance is approx)
            turn(-90.0);         //turn 90 degrees to the left
            runDistance(-10.5, true, 0.2);    //move backward (negative a distance is approx)
            grabSpecimen();      //wall state = grabbing specimen off wall
            runDistance(10, true, 0.5);     //go forwards (positive x2 distance is apporx)
            turn(-90.0);          //turn 90 degrees to the left
            runDistance(-45.5, true, 0.5);    //move backward (negative z distance is approx)
            turn(-90.0);          //turn 90 degrees to the left
            runDistance(-12, true, 0.5);    // move backward (negative y distance is approx)
            hangSpecimen(2);     //hanging specimen 2 on the chamber


        }

    }  // wait for stop


    //FUNCTIONS
    public void initializeMotion() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setTargetPosition(0);
        leftBackDrive.setTargetPosition(0);
        rightFrontDrive.setTargetPosition(0);
        rightBackDrive.setTargetPosition(0);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

    }

    public void runDistance(double distance, boolean waitForFinish, double drivePower) {
        initializeMotion();
        int varLeft = Math.abs(leftFrontDrive.getCurrentPosition());
        int Encoder = (int)(distance*45.033);
        // Output the safe vales to the motor drives.
        //double drivePower = 0.4;
        leftFrontDrive.setPower(drivePower);
        leftBackDrive.setPower(drivePower);
        rightFrontDrive.setPower(drivePower);
        rightBackDrive.setPower(drivePower);
        leftFrontDrive.setTargetPosition(Encoder);
        leftBackDrive.setTargetPosition(Encoder);
        rightFrontDrive.setTargetPosition(Encoder);
        rightBackDrive.setTargetPosition(Encoder);

        while(varLeft < Math.abs(Encoder) && waitForFinish == true ){
            varLeft = Math.abs(leftFrontDrive.getCurrentPosition());
        }

    }
    public void telemetry(){
        telemetry.addData("leftfront %f", leftFrontDrive.getCurrentPosition() );
        telemetry.addData("leftback %f", leftBackDrive.getCurrentPosition() );
        telemetry.addData("rightfront %f", rightFrontDrive.getCurrentPosition() );
        telemetry.addData("rightback %f", rightBackDrive.getCurrentPosition() );
        telemetry.addData("Vertical Left Motor Position", verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical Right Motor Position", verticalRight.getCurrentPosition());
        telemetry.update();
    }

    public void turn(double degrees) {
        // Output the safe vales to the motor drives.
        int lessEncoder = (int)(Math.abs(degrees)*(11457.5/1800)); //change 20 to actual number
        int moreEncoder = (int)(Math.abs(degrees)*(12185.5/1800));

        initializeMotion();

        double turnPower = 0.2;
        int varLeft = Math.abs(leftFrontDrive.getCurrentPosition());
        int varRight = Math.abs(rightFrontDrive.getCurrentPosition());

        if (degrees > 0) { // positive degrees = turn right

            leftFrontDrive.setPower(-turnPower);
            leftBackDrive.setPower(-turnPower);
            rightFrontDrive.setPower(turnPower);
            rightBackDrive.setPower(turnPower);

            leftFrontDrive.setTargetPosition(-lessEncoder);
            leftBackDrive.setTargetPosition(-lessEncoder);
            rightFrontDrive.setTargetPosition(moreEncoder);
            rightBackDrive.setTargetPosition(moreEncoder);

            while(varLeft < lessEncoder && varRight < moreEncoder){
                telemetry();
                varLeft = Math.abs(leftFrontDrive.getCurrentPosition());
                varRight = Math.abs(rightFrontDrive.getCurrentPosition());
            }

        }
        else {             // negative degres = turn left
            leftFrontDrive.setPower(turnPower);
            leftBackDrive.setPower(turnPower);
            rightFrontDrive.setPower(-turnPower);
            rightBackDrive.setPower(-turnPower);

            leftFrontDrive.setTargetPosition(moreEncoder);
            leftBackDrive.setTargetPosition(moreEncoder);
            rightFrontDrive.setTargetPosition(-lessEncoder);
            rightBackDrive.setTargetPosition(-lessEncoder);
            telemetry();
            //sleep(2000);

            while(varLeft < moreEncoder && varRight < lessEncoder){
                varLeft = Math.abs(leftFrontDrive.getCurrentPosition());
                varRight = Math.abs(rightFrontDrive.getCurrentPosition());
                telemetry();
            }
        }
    }
    public void hangSpecimen(int specimenNumber){
        chamber(false, 1);
        telemetry.addData("Status: Hanging Specimen %d", specimenNumber);
        telemetry.update();
        sleep(1500);
        runDistance(-6.0, true, 0.2);
        chamber(false, 2);
        sleep(1000);
        chamber(true, 0);
        sleep(1000);
    }

    public void chamber(boolean open, double extended) {
        if (open == true){
            clawservoe1 = ClawServoE.CLAW_OPEN;
        }
        else{
            clawservoe1 = ClawServoE.CLAW_CLOSE;
        }

        if (clawservoe1 == ClawServoE.CLAW_CLOSE)
            clawservo.setPosition(1.2); // vertical extension claw is closed
        if (clawservoe1 == ClawServoE.CLAW_OPEN)
            clawservo.setPosition(0); // vertical extension claw is open


        //else if (extended == true && gamepad1.a == true) {
        //    attach = true;
        //    extended = false;

        //}
        if (extended == 0){
            vertchame = VertE.RESET_CHAM;
        }
        if (extended == 1){
            vertchame = VertE.ABOVE_CHAM;
        }
        if (extended == 2){
            vertchame = VertE.ON_CHAM;
        }


        // hardware calls
        if (vertchame == VertE.ABOVE_CHAM) {
            verticalLeft.setTargetPosition(1300);
            verticalRight.setTargetPosition(-1300);
        } else if (vertchame == VertE.ON_CHAM) {
            verticalLeft.setTargetPosition(900);
            verticalRight.setTargetPosition(-900);
        } else if (vertchame == VertE.RESET_CHAM) {
            verticalLeft.setTargetPosition(0);
            verticalRight.setTargetPosition(0);
        }

    }
    public void grabSpecimen() {
        sleep(100);
        wall(true, false); //claw = open. vertical slides = down
        sleep(500);
        wall(false, false); //claw = close. vertical slides = down
        sleep(500);
        wall(false, true);  //claw = close. vertical slides = up
        sleep(500);

    }


    public void wall(boolean open, boolean extended_vert) {
        if (open == true){
            clawservoe1 = ClawServoE.CLAW_OPEN;
        }
        else{
            clawservoe1 = ClawServoE.CLAW_CLOSE;
        }

        if (clawservoe1 == ClawServoE.CLAW_CLOSE) {
            clawservo.setPosition(1.2); //claw is closed
        }
        if (clawservoe1 == ClawServoE.CLAW_OPEN) {
            clawservo.setPosition(0); //claw is open
        }

        if (extended_vert == true) {
            verticalLeft.setTargetPosition(300);
            verticalRight.setTargetPosition(-300);
        } else if (extended_vert == false) {
            verticalLeft.setTargetPosition(0);
            verticalRight.setTargetPosition(0);
        }

        telemetry.addData("ClawServoPOS!", clawservo.getPosition());
        telemetry.update();

        telemetry.addData("Vertical Left Motor Position", verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical Right Motor Position", verticalRight.getCurrentPosition());

        telemetry.update();

    }
    public void arc(double radiusInInches) {
        int radiusInTicks = (int)(radiusInInches*14); //**14 is a placeholder number**

        double lessPower = 0.2;
        double morePower = 0.3;
        if (radiusInInches > 0){ // Positive radius = Turn Left (Assumption)
            leftFrontDrive.setPower(lessPower);
            leftBackDrive.setPower(lessPower);
            rightFrontDrive.setPower(morePower);
            rightBackDrive.setPower(morePower);
        }
        else {               // Negative radius = Turn Right (Assumption)
            leftFrontDrive.setPower(morePower);
            leftBackDrive.setPower(morePower);
            rightFrontDrive.setPower(lessPower);
            rightBackDrive.setPower(lessPower);
        }
        leftFrontDrive.setTargetPosition(radiusInTicks);
        leftBackDrive.setTargetPosition(radiusInTicks);
        rightFrontDrive.setTargetPosition(radiusInTicks);
        rightBackDrive.setTargetPosition(radiusInTicks);
    }
}

