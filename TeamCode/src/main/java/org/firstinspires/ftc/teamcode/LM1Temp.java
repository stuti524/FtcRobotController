package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.CRServo;
//import org.firstinspires.ftc.teamcode.Sample;
//import org.firstinspires.ftc.teamcode.Wall;
//import org.firstinspires.ftc.teamcode.Drive;
//import org.firstinspires.ftc.teamcode.Chamber;
@Disabled
@TeleOp(name = "LM1Temp")
public class LM1Temp extends LinearOpMode {
    enum StateE {
        SAMPLE_STATE,
        CHAMBER_STATE,
        WALL_STATE,
        DRIVE_STATE
    };
    enum VertE {
        ABOVE_CHAM,
        ON_CHAM,
        RESET_CHAM
    };
    StateE state = StateE.CHAMBER_STATE;

    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;

    public double left = 0;
    public double right = 0;
    public double drive = 0;
    public double turn = 0;
    public double max = 0;

    //Sample
    // HORIZONTAL CODE - TEMP - Improve it with STATE MACHINE or DIFFERNET EFFIVIENT LOGIC
    public DcMotor horizontalMotor1 = null;
    public DcMotor verticalLeft1 = null;
    public DcMotor verticalRight1 = null;

    public boolean rightArrow1 = false;
    public boolean leftArrow1 = false;

    public boolean extended_hori = false;
    public boolean extended_vert = false;

    enum RotateServoE {
        //INTAKE_NO_DOWN_UP,
        INTAKE_DOWN,
        INTAKE_UP
    };

    //SERVO CODE
    Servo rotateServo;
    //Sample.RotateServoE rotateServoe= Sample.RotateServoE.INTAKE_UP;
    //clawServo
    Servo clawservo;
    //ClawServo.ClawServoE clawservoe= ClawServo.ClawServoE.CLAW_CLOSE;
    //SPING SERVO
    CRServo spinServo;
    //Chamber.VertE vertchame= Chamber.VertE.RESET_CHAM;
    //Wall

    enum ClawServoE {
        //INTAKE_NO_DOWN_UP,
        CLAW_CLOSE,
        CLAW_OPEN
    };

    Servo clawservo1;
    ClawServoE clawservoe1= ClawServoE.CLAW_CLOSE;

    @Override
    public void runOpMode() {

        // Define and Initialize Motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "cm2");
        leftBackDrive = hardwareMap.get(DcMotor.class, "cm3");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "cm0");
        rightBackDrive = hardwareMap.get(DcMotor.class, "cm1");

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        //Sample

        //VERTICAL

        horizontalMotor1 = hardwareMap.get(DcMotor.class, "em0");
        horizontalMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotateServo = hardwareMap.get(Servo.class, "es3");

        //SPIN SERVO
        spinServo = hardwareMap.get(CRServo.class, "es5");

        //SPING SERVO
        spinServo.setPower(0);

        telemetry.addLine("Ready to Drive - Press Start!");
        telemetry.update();

        //Wall
        clawservo1 = hardwareMap.get(Servo.class, "cs5");

        verticalLeft1 = hardwareMap.get(DcMotor.class, "em1");
        verticalRight1 = hardwareMap.get(DcMotor.class, "em2");

        verticalLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        //Wall

        verticalLeft1.setTargetPosition(0);
        verticalRight1.setTargetPosition(0);

        verticalLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        verticalLeft1.setPower(0.5);
        verticalRight1.setPower(0.5);



        while (opModeIsActive()) {
            if (gamepad1.x == true) {
                state = StateE.CHAMBER_STATE;
            } else if (gamepad1.y == true) {
                state = StateE.SAMPLE_STATE;
            } else if (gamepad1.a == true) {
                state = StateE.DRIVE_STATE;
            } else if (gamepad1.b == true) {
                state = StateE.WALL_STATE;
            }

            if (state == StateE.SAMPLE_STATE) {
                //drive();
                sample();
            }
            if (state == StateE.CHAMBER_STATE) {
                drive();
                chamber();
            }
            if (state == StateE.WALL_STATE) {
                drive();
                wall();
            }
            if (state == StateE.DRIVE_STATE) {
                drive();
            }
        }
    }

    public void drive() {
        //Drive

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

        telemetry.addData("drive %f", drive);
        telemetry.addData("turn %f", turn );
        telemetry.update();

    }
    public void sample() {
        // HORIZONTAL CODE - TEMP - Improve it with STATE MACHINE or DIFFERNET EFFICIENT LOGIC

        boolean gamepaddpad_right_hori = gamepad1.dpad_right;
        boolean gamepaddpad_left_hori = gamepad1.dpad_left;

        //telemetry.addData("Horizontal Curr Position1 %f gamepad1.dpad_right %f", String.valueOf(horizontalMotor1.getCurrentPosition()),gamepaddpad_right_hori,rightArrow1 );
        //telemetry.update();

        if (rightArrow1 == false && gamepaddpad_right_hori == true) {
            rightArrow1 = true;
            horizontalMotor1.setTargetPosition(-3000);
            //telemetry.addData("leftFrontPos Reached! %f", leftFrontDrive.getCurrentPosition());
            //telemetry.update();

        }
        if (rightArrow1 == true && gamepaddpad_right_hori == false) {
            rightArrow1 = false;
            //telemetry.addData("leftFrontPos FALSE! %f", leftFrontDrive.getCurrentPosition());
            //telemetry.update();
        }

        if (leftArrow1 == false && gamepaddpad_left_hori == true) {
            leftArrow1 = true;
            horizontalMotor1.setTargetPosition(0);

        }
        if (leftArrow1 == true && gamepaddpad_left_hori == false) {
            leftArrow1 = false;
        }

        //telemetry.addData("leftFrontPos %f", leftFrontDrive.getCurrentPosition());
        telemetry.addData("Right_Hori", gamepaddpad_right_hori);
        telemetry.update();
        //telemetry.addData("Horizontal Curr Position %f gamepad1.dpad_right %f", String.valueOf(horizontalMotor1.getCurrentPosition()),gamepad1.dpad_right );
        telemetry.addData("RightArrow", rightArrow1);
        telemetry.update();

        //SERVO
//        //if (rotateServoe == Sample.RotateServoE.INTAKE_UP  && gamepad1.right_bumper==true)
//            //down=true;
//            rotateServoe = Sample.RotateServoE.INTAKE_DOWN;
//        else if ( rotateServoe == Sample.RotateServoE.INTAKE_DOWN && gamepad1.left_bumper==true)
//            //down=false;
//            rotateServoe = Sample.RotateServoE.INTAKE_UP;
//// hardware calls
//        if (rotateServoe == Sample.RotateServoE.INTAKE_DOWN)
//            rotateServo.setPosition(0.3);//rotating intake fully down
//        if (rotateServoe == Sample.RotateServoE.INTAKE_UP)
//            rotateServo.setPosition(1);//rotating intake fully upright

        //telemetry.addData("RotateServoPOS", rotateServo.getPosition() );

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
    public void wall() {
        if (clawservoe1 == ClawServoE.CLAW_OPEN && gamepad1.left_bumper == true)
            //down=true;
            clawservoe1 = ClawServoE.CLAW_CLOSE;
        else if (clawservoe1 == ClawServoE.CLAW_CLOSE && gamepad1.right_bumper == true)
            //down=false;
            clawservoe1 = ClawServoE.CLAW_OPEN;

        if (clawservoe1 == ClawServoE.CLAW_CLOSE)
            clawservo1.setPosition(1.2); // vertical extension claw is closed
        if (clawservoe1 == ClawServoE.CLAW_OPEN)
            clawservo1.setPosition(0); // vertical extension claw is open


        boolean gamepaddpad_up1 = gamepad1.dpad_up;
        boolean gamepaddpad_down1 = gamepad1.dpad_down;

        // business logic
        if (extended_vert == false && gamepaddpad_up1 == true) { // retracted and gamepad.y is held
            extended_vert = true;
        } else if (extended_vert == true && gamepaddpad_down1 == true) { // extended and gamepad.y is released
            extended_vert = false;
        }
        //else if (extended == true && gamepad1.a == true) {
        //    attach = true;
        //    extended = false;
        if (extended_vert == true) {
            verticalLeft1.setTargetPosition(200);
            verticalRight1.setTargetPosition(-200);
        } else if (extended_vert == false) {
            verticalLeft1.setTargetPosition(0);
            verticalRight1.setTargetPosition(0);
        }

        telemetry.addData("ClawServoPOS!", clawservo1.getPosition());
        telemetry.update();

        telemetry.addData("Vertical Left Motor Position", verticalLeft1.getCurrentPosition());
        telemetry.addData("Vertical Right Motor Position", verticalRight1.getCurrentPosition());

        telemetry.update();

    }
    public void chamber() {
        if (clawservoe1 == ClawServoE.CLAW_OPEN && gamepad1.left_bumper == true)
            //down=true;
            clawservoe1 = ClawServoE.CLAW_CLOSE;
        else if (clawservoe1 == ClawServoE.CLAW_CLOSE && gamepad1.right_bumper == true)
            //down=false;
            clawservoe1 = ClawServoE.CLAW_OPEN;

        if (clawservoe1 == ClawServoE.CLAW_CLOSE)
            clawservo1.setPosition(1.2); // vertical extension claw is closed
        if (clawservoe1 == ClawServoE.CLAW_OPEN)
            clawservo1.setPosition(0); // vertical extension claw is open

        boolean gamepaddpad_up1 = gamepad1.dpad_up;
        boolean gamepaddpad_down1 = gamepad1.dpad_down;
        boolean gamepaddpad_right1 = gamepad1.dpad_right;

//        // business logic
//        if (vertchame == Chamber.VertE.RESET_CHAM && gamepaddpad_up1 == true) { // retracted and gamepad.y is held
//            vertchame = Chamber.VertE.ABOVE_CHAM;
//        } else if (vertchame == Chamber.VertE.ABOVE_CHAM && gamepaddpad_right1 == true) { // extended and gamepad.y is released
//            vertchame = Chamber.VertE.ON_CHAM;
//        } else if(vertchame == Chamber.VertE.ON_CHAM && gamepaddpad_down1 == true) {
//            vertchame = Chamber.VertE.RESET_CHAM;
//        }
//        //else if (extended == true && gamepad1.a == true) {
//        //    attach = true;
//        //    extended = false;
//
//        //}
//
//        // hardware calls
//        if (vertchame == Chamber.VertE.ABOVE_CHAM) {
//            verticalLeft1.setTargetPosition(1300);
//            verticalRight1.setTargetPosition(-1300);
//        } else if (vertchame == Chamber.VertE.ON_CHAM) {
//            verticalLeft1.setTargetPosition(900);
//            verticalRight1.setTargetPosition(-900);
//        } else if (vertchame == Chamber.VertE.RESET_CHAM) {
//            verticalLeft1.setTargetPosition(0);
//            verticalRight1.setTargetPosition(0);
//        }
        telemetry.addData("ClawServoPOS!", clawservo1.getPosition());

        telemetry.update();

        telemetry.addData("Vertical Left Motor Position", verticalLeft1.getCurrentPosition());
        telemetry.addData("Vertical Right Motor Position", verticalRight1.getCurrentPosition());

        telemetry.update();
    }
}