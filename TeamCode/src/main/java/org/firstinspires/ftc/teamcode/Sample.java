package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


//LEFT JOYSTICK - FORWARD and BACKWARD
// RIGHT JOYSTICK - RIGHT and LEFT Turns
@Disabled
@TeleOp(name = "Sample")    
public class Sample extends LinearOpMode {
    enum RotateServoE {
        //INTAKE_NO_DOWN_UP,
        INTAKE_DOWN,
        INTAKE_UP
    };
    /* Declare OpMode members. */
    public DcMotor leftFrontDrive   = null;
    public DcMotor leftBackDrive   = null;
    public DcMotor  rightFrontDrive  = null;
    public DcMotor  rightBackDrive  = null;

    //SERVO CODE
    Servo rotateServo;
    RotateServoE rotateServoe= RotateServoE.INTAKE_UP;
    //clawServo
    Servo clawservo;
    //ClawServo.ClawServoE clawservoe= ClawServo.ClawServoE.CLAW_CLOSE;
    //SPING SERVO
    CRServo spinServo;


    @Override
    public void runOpMode() {
        //Variables for Drive
        double left=0;
        double right=0;
        double drive=0;
        double turn=0;
        double max=0;


        // HORIZONTAL CODE - TEMP - Improve it with STATE MACHINE or DIFFERNET EFFIVIENT LOGIC
        boolean rightArrow = false;
        boolean leftArrow = false;

        //VERTICAL

        DcMotor horizontalMotor = hardwareMap.get(DcMotor.class, "em0");
        horizontalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and Initialize Motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "cm2");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "cm3");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "cm0");
        rightBackDrive = hardwareMap.get(DcMotor.class, "cm1");

        rotateServo = hardwareMap.get(Servo.class, "es3");

        //SPIN SERVO
        spinServo = hardwareMap.get(CRServo.class, "es5");

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        //SPIN SERVO
        spinServo.setPower(0);

        //VERTICAL

        telemetry.addLine("Ready to Drive - Press Start!");
        telemetry.update();

        waitForStart();

        // HORIZONTAL CODE - TEMP - Improve it with STATE MACHINE or DIFFERNET EFFIVIENT LOGIC
        horizontalMotor.setTargetPosition(0);
        horizontalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalMotor.setPower(1);

        //while (opModeIsActive() && (runtime.seconds() < 2.0)) {
        //    leftFrontDrive.setPower(0.4);
        //    leftBackDrive.setPower(0.4);
        //    rightFrontDrive.setPower(0.4);
        //    rightBackDrive.setPower(0.4);
        //}
        boolean extended = false;
        boolean attach = false;

        while(opModeIsActive()){
            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;



            // Combine drive and turn for blended motion.
            //TEMP SOL - DIV by 2 to slow down the DT
            left  = (drive + turn)/1.5;
            right = (drive - turn)/1.5;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            // Output the safe vales to the motor drives.
            leftFrontDrive.setPower(left);
            leftBackDrive.setPower(left);
            rightFrontDrive.setPower(right);
            rightBackDrive.setPower(right);

            // HORIZONTAL CODE - TEMP - Improve it with STATE MACHINE or DIFFERNET EFFICIENT LOGIC

            if (rightArrow == false && gamepad1.dpad_right == true) {
                rightArrow = true;
                horizontalMotor.setTargetPosition(-3000);
            }

            if (rightArrow == true && gamepad1.dpad_right == false) {
                rightArrow = false;
            }

            if (leftArrow == false && gamepad1.dpad_left == true) {
                leftArrow = true;
                horizontalMotor.setTargetPosition(0);

            }
            if (leftArrow == true && gamepad1.dpad_left == false) {
                leftArrow = false;
            }
            telemetry.addData("leftFrontPos %f", leftFrontDrive.getCurrentPosition() );
            telemetry.addData("Horizontal Curr Position %f", horizontalMotor.getCurrentPosition() );


            //SERVO
            if (rotateServoe == RotateServoE.INTAKE_UP  && gamepad1.right_bumper==true)
                //down=true;
                rotateServoe = RotateServoE.INTAKE_DOWN;
            else if ( rotateServoe == RotateServoE.INTAKE_DOWN && gamepad1.left_bumper==true)
                //down=false;
                rotateServoe = RotateServoE.INTAKE_UP;
// hardware calls
            if (rotateServoe == RotateServoE.INTAKE_DOWN)
                rotateServo.setPosition(0.3);//rotating intake fully down
            if (rotateServoe == RotateServoE.INTAKE_UP)
                rotateServo.setPosition(1);//rotating intake fully upright

            telemetry.addData("RotateServoPOS", rotateServo.getPosition() );

            //SPIN SERVO
            if (gamepad1.left_trigger != 0 ) { // LT is pressed
                spinServo.setPower(0.9);
            }

            if (gamepad1.right_trigger !=0 ) { // RT is pressed
                spinServo.setPower(-0.9);
            }

            if ( (gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0) ){//LT and RT is not pressed
                spinServo.setPower(0);
            }
        }

    }  // wait for stop

}

