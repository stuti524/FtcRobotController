package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


//LEFT JOYSTICK - FORWARD and BACKWARD
// RIGHT JOYSTICK - RIGHT and LEFT Turns

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
            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
//            drive = -gamepad1.left_stick_y;
//            turn  =  gamepad1.right_stick_x;



            // Combine drive and turn for blended motion.
            //TEMP SOL - DIV by 2 to slow down the DT
//            left  = (drive + turn)/1.5;
//            right = (drive - turn)/1.5;
//
//            // Normalize the values so neither exceed +/- 1.0
//            max = Math.max(Math.abs(left), Math.abs(right));
//            if (max > 1.0)
//            {
//                left /= max;
//                right /= max;
//            }
            telemetry();


            // Output the safe vales to the motor drives.
            leftFrontDrive.setPower(0.2);
            leftBackDrive.setPower(0.2);
            rightFrontDrive.setPower(0.2);
            rightBackDrive.setPower(0.2);
            //Park Code
            //runDistance(37.75);

            runDistance(-29.5);
            chamber(false, 1);
            telemetry();
            telemetry.addLine("Status: Hanging Preset");
            sleep(4000);
            chamber(false, 2);
            sleep(2000);
            chamber(true, 0);
            sleep(2000);
            //runDistance(0);
            telemetry();
            //sleep(5000);
            telemetry.addLine("Turning Off");

        }

    }  // wait for stop

    public void runDistance(double distance){
        int Encoder = (int)(distance*45.033);
        leftFrontDrive.setTargetPosition(Encoder);
        leftBackDrive.setTargetPosition(Encoder);
        rightFrontDrive.setTargetPosition(Encoder);
        rightBackDrive.setTargetPosition(Encoder);
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
}

