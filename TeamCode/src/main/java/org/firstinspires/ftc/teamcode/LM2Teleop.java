package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "LM2Teleop")
public class LM2Teleop extends LinearOpMode {
    TransferFinal tf = new TransferFinal();
    enum StateE {
        SAMPLE_STATE,
        CHAMBER_STATE,
        WALL_STATE,
        ASCENT_STATE
    }

    ;

    enum VertE {
        ABOVE_BASK,
        ZERO_BASK
    }

    enum HortE {
        HORI_EXTEND,
        HORI_RETRACT
    }

    ;
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

    public boolean rightArrow = false;
    public boolean leftArrow = false;
    public boolean downArrow = false;

    public boolean extended_hori = false;
    public boolean extended_vert = false;
    public boolean positionReached = false;


    enum IntakeRotateStates {
        INTAKE_DOWN,
        INTAKE_MIDDLE,
        INTAKE_UP
    }

    enum IntakeGrabStates {
        GRAB_CLOSE,
        GRAB_OPEN,
        GRAB_ADJUST
    }

    enum IntakeGimbleStates {
        GIMBLE_CENTER,
        GIMBLE_NINETY
    }

    enum TransferRotateStates {
        TRANSFER_DOWN,
        TRANSFER_MIDDLE,
        TRANSFER_UP,
        TRANSFER_HANG
    }

    enum TransferGrabStates {
        TRANSFER_CLOSE,
        TRANSFER_OPEN,
        TRANSFER_ADJUST
    }

    enum TransferGimbleStates {
        TRANSFER_CENTER,
        TRANSFER_NINETY,
        GIMBLE_HANG,
        GIMBLE_BASKET
    }

    //SERVO CODE
    // Servo rotateServo;

    //clawServo
//    Servo clawservo;
//    ClawServoE clawservoe = ClawServoE.CLAW_CLOSE;
    VertE vertbaske = VertE.ZERO_BASK;

    HortE hortE = HortE.HORI_RETRACT;
    //Wall

//    enum ClawServoE {
//        //INTAKE_NO_DOWN_UP,
//        CLAW_CLOSE,
//        CLAW_OPEN
//    }
//
//    ;
//
//    Servo clawservo1;
//    ClawServoE clawservoe1 = ClawServoE.CLAW_CLOSE;

    enum HangE {
        ABOVE_RUNG,
        ON_RUNG,
        ZERO_RUNG,
        L3_RUNG
    }

    HangE hangE = HangE.ZERO_RUNG;
    Servo servostop1;
    Servo servostop2;

    //SERVO CODE
    Servo rotateServo;
    Servo grabServo;
    Servo gimbleServo;

    //Transfer Servos
    Servo TrotateServo;
    Servo TgrabServo;
    Servo TgimbleServo;

    //Enum Initilization:
    IntakeRotateStates rotateServoe = IntakeRotateStates.INTAKE_UP;
    IntakeGrabStates grabServoe = IntakeGrabStates.GRAB_OPEN;
    IntakeGimbleStates gimbleServoe = IntakeGimbleStates.GIMBLE_NINETY;
    TransferGrabStates TgrabServoe = TransferGrabStates.TRANSFER_CLOSE;
    TransferGimbleStates TgimbleServoe = TransferGimbleStates.GIMBLE_HANG;
    TransferRotateStates TrotateServoe = TransferRotateStates.TRANSFER_HANG;



    boolean rightBumper = false;
    boolean leftBumper = false;
    boolean dpadUp = false;
    boolean upArrow = false;

    @Override

    public void runOpMode() {
        //CLAW INTAKE
        rotateServo = hardwareMap.get(Servo.class, "es3");
        grabServo = hardwareMap.get(Servo.class, "es5");
        gimbleServo = hardwareMap.get(Servo.class, "es1");

        //FOR ASCENT
        servostop1 = hardwareMap.get(Servo.class, "cs3");
        servostop2 = hardwareMap.get(Servo.class, "cs1");

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

        tf.servoInitializationTeleop(hardwareMap,0.7);

        //Sample

        //VERTICAL

        horizontalMotor1 = hardwareMap.get(DcMotor.class, "em0");
        horizontalMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalMotor1.setTargetPosition(0);
        horizontalMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalMotor1.setPower(1);

//        rotateServo = hardwareMap.get(Servo.class, "es3");

        telemetry.addLine("Ready to Drive - Press Start!");
        telemetry.update();

        //Wall
//Transfer Servos
        TrotateServo = hardwareMap.get(Servo.class, "cs5");
        TgrabServo = hardwareMap.get(Servo.class, "cs2");
        TgimbleServo = hardwareMap.get(Servo.class, "cs4");

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

        verticalLeft1.setPower(0.7);
        verticalRight1.setPower(0.7);


        while (opModeIsActive()) {
            if (state != StateE.SAMPLE_STATE) {
                rotateServo.setPosition(1);//rotating intake fully upright
                grabServo.setPosition(1);
                gimbleServo.setPosition(0.275);
            }

            if (state != StateE.ASCENT_STATE) {
                servostop1.setPosition(1.0);
                servostop2.setPosition(0.20);
            }
            if (gamepad1.x == true) {
                state = StateE.CHAMBER_STATE;
            } else if (gamepad1.y == true) {
                state = StateE.SAMPLE_STATE;
            } else if (gamepad1.a == true) {
                state = StateE.ASCENT_STATE;
            } else if (gamepad1.b == true) {
                state = StateE.WALL_STATE;
            }

            if (state == StateE.SAMPLE_STATE) {
                drive();
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
            if (state == StateE.ASCENT_STATE) {
                //boolean GPLB = gamepad1.left_bumper;
                drive();
                ascent();
                while (gamepad1.left_bumper) {
                    ascentFinal();
                    //GPLB = gamepad1.left_bumper;
                }
            }
        }
    }

    public void ascent() {
        Servo servostop1 = hardwareMap.get(Servo.class, "cs3");
        Servo servostop2 = hardwareMap.get(Servo.class, "cs1");
        boolean GPRB = gamepad1.right_bumper;
//        clawservo1.setPosition(0);
        if (GPRB && hangE == HangE.ZERO_RUNG) {
            hangE = HangE.ABOVE_RUNG;
        }
        //Hardware calls
        if (hangE == HangE.ABOVE_RUNG) {
            verticalLeft1.setTargetPosition(1500);
            verticalRight1.setTargetPosition(-1500);
            TrotateServo.setPosition(0.6);
            TgimbleServo.setPosition(1);
            //THESE SERVO POS WORK
            servostop1.setPosition(0);
            servostop2.setPosition(0.6);
        }

        telemetry.addData("vertical left %f", verticalLeft1.getCurrentPosition());
        telemetry.addData("vertical right %f", verticalRight1.getCurrentPosition());
        telemetry.update();
    }

    public void ascentFinal() {
        boolean GPRB = gamepad1.right_bumper;
        boolean GPRA = gamepad1.dpad_right;
       //Business Logic
        if (GPRB && hangE == HangE.ABOVE_RUNG) {
            hangE = HangE.ON_RUNG;
        }
        if (GPRA && hangE == HangE.ON_RUNG) {
           hangE = HangE.L3_RUNG;
        }
        //L2
        if (hangE == HangE.ON_RUNG) {
            verticalLeft1.setTargetPosition(0);
            verticalRight1.setTargetPosition(0);
            sleep(500);
//            verticalLeft1.setTargetPosition(300);
//            verticalRight1.setTargetPosition(-300);
//
//            sleep(500);
        }
        telemetry.addData("rightback", rightBackDrive.getCurrentPosition());
        telemetry.addData("leftback", leftBackDrive.getCurrentPosition());
        telemetry.addData("vertical left %f", verticalLeft1.getCurrentPosition());
        telemetry.addData("vertical right %f", verticalRight1.getCurrentPosition());
        telemetry.update();

        //L3
//        if(hangE==HangE.L3_RUNG) {
//            verticalLeft1.setTargetPosition(200);
//            verticalRight1.setTargetPosition(-200);
//            sleep(250);
//            verticalLeft1.setTargetPosition(300);
//            verticalRight1.setTargetPosition(-300);
//            sleep(250);
//            //move up the slides
//            servostop1.setPosition(1.0);
//            servostop2.setPosition(0.20);
//            //close the micro-servos
//            sleep(100);
//            verticalLeft1.setTargetPosition(1000);
//            verticalRight1.setTargetPosition(-1000);
//            sleep(250);
//            //raise up the slides
//            servostop1.setPosition(0);
//            servostop2.setPosition(0.8);
//            //open the micro-servos again
//            sleep(100);
//            //time for L3!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//            verticalLeft1.setTargetPosition(0);
//            verticalRight1.setTargetPosition(-0);
//        }
    }

    public void drive() {
        //Drive

        // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        // This way it's also easy to just drive straight, or just turn.
        drive = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;

        // Combine drive and turn for blended motion.
        //TEMP SOL - DIV by 2 to slow down the DT
        left = (drive + turn) / 1.5;
        right = (drive - turn) / 1.5;

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        // Output the safe vales to the motor drives.
        leftFrontDrive.setPower(left);
        leftBackDrive.setPower(left);
        rightFrontDrive.setPower(right);
        rightBackDrive.setPower(right);

        //telemetry.addData("drive %f", drive);
        //telemetry.addData("turn %f", turn);
        //telemetry.update();

    }

    public void sample() {

        if (gamepad1.dpad_left) {
            tf.sampleTransfer();
        }

        if (rightArrow == false && gamepad1.dpad_right == true && hortE == HortE.HORI_RETRACT) {
            hortE = HortE.HORI_EXTEND;
        }
        else if (rightArrow == false && gamepad1.dpad_right == true && hortE == HortE.HORI_EXTEND) {
            hortE = HortE.HORI_RETRACT;
        }
        rightArrow = gamepad1.dpad_right;

        if (hortE == HortE.HORI_EXTEND) {
            horizontalMotor1.setTargetPosition(-3000);
//            // wait until pos is reached
//            while(horizontalMotor1.getCurrentPosition() < -3000)
//                ;
//            positionReached = true;
        }
        if (hortE == HortE.HORI_RETRACT) {
            horizontalMotor1.setTargetPosition(0);
        }
        //CLAW INTAKE CODE

        //ROTATE SERVO
        //MACRO
        if (gamepad1.dpad_down) {
            grabServoe = IntakeGrabStates.GRAB_OPEN;
            sleep(250);
            rotateServoe = IntakeRotateStates.INTAKE_DOWN;
            rotateServo.setPosition(0.1);
            sleep(250);
            grabServoe = IntakeGrabStates.GRAB_CLOSE;
            grabServo.setPosition(1);//close claw fully
            sleep(250);
            rotateServoe = IntakeRotateStates.INTAKE_UP;
            rotateServo.setPosition(1);//rotating intake fully upright
            sleep(500);
        }
        //business logic
        if (rotateServoe == IntakeRotateStates.INTAKE_UP && gamepad1.right_bumper == true && rightBumper == false) {
            rotateServoe = IntakeRotateStates.INTAKE_MIDDLE;
        } else if (rotateServoe == IntakeRotateStates.INTAKE_MIDDLE && gamepad1.right_bumper == true && rightBumper == false) {
            rotateServoe = IntakeRotateStates.INTAKE_DOWN;
        } else if (rotateServoe == IntakeRotateStates.INTAKE_DOWN && gamepad1.right_bumper == true && rightBumper == false) {
            rotateServoe = IntakeRotateStates.INTAKE_UP;
        }

        rightBumper = gamepad1.right_bumper;

        // hardware calls
        if (rotateServoe == IntakeRotateStates.INTAKE_DOWN)
            rotateServo.setPosition(0.1);//rotating intake fully down
        if (rotateServoe == IntakeRotateStates.INTAKE_UP)
            rotateServo.setPosition(1);//rotating intake fully upright
        if (rotateServoe == IntakeRotateStates.INTAKE_MIDDLE)
            rotateServo.setPosition(0.225);//rotate intake middle position
        telemetry.addData("RotateServoPOS", rotateServo.getPosition());
        telemetry.update();


        //GRAB SERVO

        //business logic
        if (grabServoe == IntakeGrabStates.GRAB_OPEN && gamepad1.left_bumper == true && leftBumper == false) {
            grabServoe = IntakeGrabStates.GRAB_CLOSE;
        } else if (grabServoe == IntakeGrabStates.GRAB_CLOSE && gamepad1.left_bumper == true && leftBumper == false) {
            grabServoe = IntakeGrabStates.GRAB_OPEN;
        }
        leftBumper = gamepad1.left_bumper;

        //hardware calls
        if (grabServoe == IntakeGrabStates.GRAB_CLOSE)
            grabServo.setPosition(1);//close claw fully

        if (grabServoe == IntakeGrabStates.GRAB_OPEN)
            grabServo.setPosition(0);//open claw fully


        telemetry.addData("GrabServoPOS", grabServo.getPosition());

        //GIMBLE SERVO

        //business logic
        if (gimbleServoe == IntakeGimbleStates.GIMBLE_CENTER && gamepad1.dpad_up == true && dpadUp == false) {
            gimbleServoe = IntakeGimbleStates.GIMBLE_NINETY;
        } else if (gimbleServoe == IntakeGimbleStates.GIMBLE_NINETY && gamepad1.dpad_up == true && dpadUp == false) {
            gimbleServoe = IntakeGimbleStates.GIMBLE_CENTER;
        }

        dpadUp = gamepad1.dpad_up;

        // hardware calls
        if (gimbleServoe == IntakeGimbleStates.GIMBLE_CENTER)
            gimbleServo.setPosition(0.625);
        if (gimbleServoe == IntakeGimbleStates.GIMBLE_NINETY)
            gimbleServo.setPosition(0.275);
    }
    public void controlIntake(IntakeRotateStates rotate, IntakeGrabStates grab, IntakeGimbleStates gimble) {
        //Hardware Calls for Intake Servos:
        //Intake Rotate Servo:
        if (rotate == IntakeRotateStates.INTAKE_DOWN)
            rotateServo.setPosition(0.15);//rotating intake fully down
        if (rotate == IntakeRotateStates.INTAKE_UP)
            rotateServo.setPosition(0.95);//rotating intake fully upright
        if (rotate == IntakeRotateStates.INTAKE_MIDDLE)
            rotateServo.setPosition(0.35);//rotate intake middle position
        sleep(150);

        //Intake Grab Servo:
        if (grab == IntakeGrabStates.GRAB_CLOSE)
            grabServo.setPosition(1);//close claw fully

        if (grab == IntakeGrabStates.GRAB_OPEN)
            grabServo.setPosition(0);//open claw fully

        if (grab == IntakeGrabStates.GRAB_ADJUST) {
            grabServo.setPosition(0.5);
            sleep(150);
            grabServo.setPosition(1);
        }
        sleep(150);

        //Intake Gimble Servo:
        if (gimble == IntakeGimbleStates.GIMBLE_CENTER)
            gimbleServo.setPosition(0.55);

        if (gimble == IntakeGimbleStates.GIMBLE_NINETY)
            gimbleServo.setPosition(1);
        sleep(150);
    }
    public void controlTransfer(TransferRotateStates rotate, TransferGrabStates grab, TransferGimbleStates gimble) {
        // Hardware Calls for Transfer Servos:
        // Transfer Rotate Servo:
        if (rotate == TransferRotateStates.TRANSFER_DOWN)
            TrotateServo.setPosition(1);//rotating intake fully down
        if (rotate == TransferRotateStates.TRANSFER_UP)
            TrotateServo.setPosition(0);//rotating intake fully upright
        if (rotate == TransferRotateStates.TRANSFER_MIDDLE)
            TrotateServo.setPosition(0.3);//rotate intake middle position
        if (rotate == TransferRotateStates.TRANSFER_HANG)
            TrotateServo.setPosition(0.625);//rotate intake middle position
        sleep(150);

        //Transfer Grab Servo:
        if (grab == TransferGrabStates.TRANSFER_CLOSE) {
            TgrabServo.setPosition(0); //Close Claw
            sleep(100);
            grabServo.setPosition(0);
        }
        if (grab == TransferGrabStates.TRANSFER_OPEN){
            TgrabServo.setPosition(1);//Open Claw
        }
        if (grab == TransferGrabStates.TRANSFER_ADJUST){
            TgrabServo.setPosition(0.4);//Open Claw
        }
        sleep(150);

        //Transfer Gimble Servo:
        if (gimble == TransferGimbleStates.TRANSFER_CENTER){
            TgimbleServo.setPosition(0.7);
        }
        if (gimble == TransferGimbleStates.TRANSFER_NINETY){
            TgimbleServo.setPosition(1);
        }
        if (gimble == TransferGimbleStates.GIMBLE_HANG){
            TgimbleServo.setPosition(0.8);
        }
        sleep(150);

    }
//    public void sampleTransfer(){
//        // 1. Position the intake claw right above the sample
//        // controlIntake(IntakeRotateStates.INTAKE_MIDDLE, IntakeGrabStates.GRAB_OPEN, IntakeGimbleStates.GIMBLE_NINETY);
//        // //sleep(250);
//        // // 2. Pickup the sample with the intake claw
//        // controlIntake(IntakeRotateStates.INTAKE_DOWN, IntakeGrabStates.GRAB_CLOSE, IntakeGimbleStates.GIMBLE_NINETY);
//        //sleep(250);
//        // 3. Position the rotate servo to the correct position
//        controlIntake(IntakeRotateStates.INTAKE_UP, IntakeGrabStates.GRAB_CLOSE, IntakeGimbleStates.GIMBLE_NINETY);
//        // sleep(250);
//        // 4. Loose power to position the sample correctly in the claw
//        controlIntake(IntakeRotateStates.INTAKE_UP, IntakeGrabStates.GRAB_ADJUST, IntakeGimbleStates.GIMBLE_NINETY);
//        //sleep(250);
//        // 5. Bring the transfer directly over the sample
//        controlTransfer(TransferRotateStates.TRANSFER_DOWN, TransferGrabStates.TRANSFER_OPEN, TransferGimbleStates.TRANSFER_NINETY);
//        sleep(500);
//        // 6. Grab the sample with the transfer claw
//        controlTransfer(TransferRotateStates.TRANSFER_DOWN, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.TRANSFER_NINETY);
//        //sleep(250);
//        // 7. Rotate the transfer while holding the sample
//        controlTransfer(TransferRotateStates.TRANSFER_UP, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.TRANSFER_NINETY);
//        grabServoe =IntakeGrabStates.GRAB_OPEN;
//        rotateServoe = IntakeRotateStates.INTAKE_UP;
//        gimbleServoe = IntakeGimbleStates.GIMBLE_CENTER;
//        TgrabServoe = TransferGrabStates.TRANSFER_CLOSE;
//        TrotateServoe = TransferRotateStates.TRANSFER_UP;
//        TgimbleServoe = TransferGimbleStates.TRANSFER_NINETY;
//    }
    public void wall() {
//        if (clawservoe1 == ClawServoE.CLAW_OPEN && gamepad1.left_bumper == true)
//            //down=true;
//            clawservoe1 = ClawServoE.CLAW_CLOSE;
//        else if (clawservoe1 == ClawServoE.CLAW_CLOSE && gamepad1.right_bumper == true)
//            //down=false;
//            clawservoe1 = ClawServoE.CLAW_OPEN;
//
//        if (clawservoe1 == ClawServoE.CLAW_CLOSE)
//            clawservo1.setPosition(1.2); // vertical extension claw is closed
//        if (clawservoe1 == ClawServoE.CLAW_OPEN)
//            clawservo1.setPosition(0); // vertical extension claw is open


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

        //telemetry.addData("ClawServoPOS!", clawservo1.getPosition());
        //telemetry.update();

        telemetry.addData("Vertical Left Motor Position", verticalLeft1.getCurrentPosition());
        telemetry.addData("Vertical Right Motor Position", verticalRight1.getCurrentPosition());

        telemetry.update();

    }

    public void chamber() {
//        if (clawservoe1 == ClawServoE.CLAW_OPEN && gamepad1.left_bumper == true)
//            //down=true;
//            clawservoe1 = ClawServoE.CLAW_CLOSE;
//        else if (clawservoe1 == ClawServoE.CLAW_CLOSE && gamepad1.right_bumper == true)
//            //down=false;
//            clawservoe1 = ClawServoE.CLAW_OPEN;
//
//        if (clawservoe1 == ClawServoE.CLAW_CLOSE)
//            clawservo1.setPosition(1.2); // vertical extension claw is closed
//        if (clawservoe1 == ClawServoE.CLAW_OPEN)
//            clawservo1.setPosition(0); // vertical extension claw is open

        boolean gamepaddpad_up1 = gamepad1.dpad_up;
//        boolean gamepaddpad_down1 = gamepad1.dpad_down;
//        boolean gamepaddpad_right1 = gamepad1.dpad_right;

        // business logic
//        while (gamepad1.dpad_up == true) {
//            controlTransfer(TransferRotateStates.TRANSFER_MIDDLE, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.TRANSFER_CENTER);
//            verticalLeft1.setTargetPosition(1500);
//            verticalRight1.setTargetPosition(-1500);// retracted and gamepad.y is held
//            sleep(800);
//            controlTransfer(TransferRotateStates.TRANSFER_MIDDLE, TransferGrabStates.TRANSFER_OPEN, TransferGimbleStates.TRANSFER_CENTER);
//            verticalLeft1.setTargetPosition(0);
//            verticalRight1.setTargetPosition(0);
//            controlTransfer(TransferRotateStates.TRANSFER_DOWN, TransferGrabStates.TRANSFER_OPEN, TransferGimbleStates.TRANSFER_CENTER);
//        }

        if (vertbaske == VertE.ZERO_BASK && gamepad1.dpad_up == true && upArrow == false) {
            vertbaske = VertE.ABOVE_BASK;
        }
        else if (vertbaske == VertE.ABOVE_BASK && gamepad1.dpad_up == true && upArrow == false) {
            vertbaske = VertE.ZERO_BASK;
        }
        upArrow = gamepad1.dpad_up;

        if (vertbaske == VertE.ABOVE_BASK) {
            verticalLeft1.setTargetPosition(2100);
            verticalRight1.setTargetPosition(-2100);
            sleep(400);
            TrotateServoe = TransferRotateStates.TRANSFER_MIDDLE;
            TrotateServo.setPosition(0.3);//rotate
            TgimbleServoe = TransferGimbleStates.GIMBLE_BASKET;
            TgimbleServo.setPosition(0.76);

        }
        if (vertbaske == VertE.ZERO_BASK) {
            verticalLeft1.setTargetPosition(0);
            verticalRight1.setTargetPosition(0);
        }
//        } else if (vertchame == VertE.ABOVE_CHAM && gamepaddpad_right1 == true) { // extended and gamepad.y is released
//            vertchame = VertE.ON_CHAM;
//        } else if(vertchame == VertE.ON_CHAM && gamepaddpad_down1 == true) {
//            vertchame = VertE.RESET_CHAM;
//        }
        //else if (extended == true && gamepad1.a == true) {
        //    attach = true;
        //    extended = false;

        //MACRO
        if (gamepad1.dpad_right == true) {
            hangSpecimen();
        }
        if (gamepad1.dpad_left == true) {
            specimenPickup();
        }
        if (gamepad1.dpad_down == true) {
            finishHang();
        }
        //}

        // hardware calls
//        if (vertchame == VertE.ABOVE_CHAM) {
//            verticalLeft1.setTargetPosition(1300);
//            verticalRight1.setTargetPosition(-1300);
//        } else if (vertchame == VertE.ON_CHAM) {
//            verticalLeft1.setTargetPosition(900);
//            verticalRight1.setTargetPosition(-900);
//        } else if (vertchame == VertE.RESET_CHAM) {
//            verticalLeft1.setTargetPosition(0);
//            verticalRight1.setTargetPosition(0);
//        }
       // telemetry.addData("ClawServoPOS!", clawservo1.getPosition());

       // telemetry.update();

        telemetry.addData("Vertical Left Motor Position", verticalLeft1.getCurrentPosition());
        telemetry.addData("Vertical Right Motor Position", verticalRight1.getCurrentPosition());

        telemetry.update();

        // // Business Logic for Transfer Servos:
        //     // Transfer Rotate Servo:
            if (TrotateServoe == TransferRotateStates.TRANSFER_UP && gamepad1.right_bumper == true && rightBumper == false) {
                 TrotateServoe = TransferRotateStates.TRANSFER_MIDDLE;
             } else if (TrotateServoe == TransferRotateStates.TRANSFER_MIDDLE && gamepad1.right_bumper == true && rightBumper == false) {
                 TrotateServoe = TransferRotateStates.TRANSFER_DOWN;
             } else if (TrotateServoe == TransferRotateStates.TRANSFER_DOWN && gamepad1.right_bumper == true && rightBumper == false) {
                 TrotateServoe = TransferRotateStates.TRANSFER_UP;
             }

            rightBumper = gamepad1.right_bumper;

        //     // Transfer Grab Servo:
             if (TgrabServoe == TransferGrabStates.TRANSFER_OPEN && gamepad1.left_bumper == true && leftBumper == false) {
                 TgrabServoe = TransferGrabStates.TRANSFER_CLOSE;
             }
             else if (TgrabServoe == TransferGrabStates.TRANSFER_CLOSE && gamepad1.left_bumper == true && leftBumper == false) {
                 TgrabServoe = TransferGrabStates.TRANSFER_OPEN;
             }

             leftBumper = gamepad1.left_bumper;

        if (TgrabServoe == TransferGrabStates.TRANSFER_CLOSE) {
            TgrabServo.setPosition(0);//close claw fully
        }

        if (TgrabServoe == TransferGrabStates.TRANSFER_OPEN) {
            TgrabServo.setPosition(1);//close claw fully
        }

        if (TrotateServoe == TransferRotateStates.TRANSFER_DOWN) {
            TrotateServo.setPosition(1);//rotating intake fully down
        }

        if (TrotateServoe == TransferRotateStates.TRANSFER_UP) {
            TrotateServo.setPosition(0);//rotating intake fully upright
        }

        if (TrotateServoe == TransferRotateStates.TRANSFER_MIDDLE) {
            TrotateServo.setPosition(0.3);//rotate intake middle position
        }

        //     //Transfer Gimble Servo:
//             if (TgimbleServoe == TransferGimbleStates.TRANSFER_CENTER && gamepad1.dpad_up == true && dpadUp == false) {
//                 TgimbleServoe = TransferGimbleStates.TRANSFER_NINETY;
//             }
//             else if (TgimbleServoe == TransferGimbleStates.TRANSFER_NINETY && gamepad1.dpad_up == true && dpadUp == false) {
//                 TgimbleServoe = TransferGimbleStates.TRANSFER_CENTER;
//             }

    }
    public void specimenPickup(){
        // 1. Initialization
        //controlTransfer(TransferRotateStates.TRANSFER_DOWN, TransferGrabStates.TRANSFER_OPEN, TransferGimbleStates.TRANSFER_NINETY);
        // 2. Orient the transfer into the correct position
        controlTransfer(TransferRotateStates.TRANSFER_UP, TransferGrabStates.TRANSFER_OPEN, TransferGimbleStates.TRANSFER_CENTER);
        // 2. Grab the specimen
        controlTransfer(TransferRotateStates.TRANSFER_UP, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.TRANSFER_CENTER);
        verticalLeft1.setTargetPosition(300);
        verticalRight1.setTargetPosition(-300);
        sleep(400);
        TgrabServoe = TransferGrabStates.TRANSFER_CLOSE;
        TrotateServoe = TransferRotateStates.TRANSFER_UP;
        TgimbleServoe = TransferGimbleStates.TRANSFER_CENTER;

        // 3. Rotate the transfer back
        // controlTransfer(TransferRotateStates.TRANSFER_DOWN, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.TRANSFER_CENTER);
    }

    public void hangSpecimen(){
        //1. Intialize transfer in the up position
        //controlTransfer(TransferRotateStates.TRANSFER_UP, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.TRANSFER_CENTER);
        //2. Raise vertical while transfer rotates into down position
        controlTransfer(TransferRotateStates.TRANSFER_HANG, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.GIMBLE_HANG);
        controlTransfer(TransferRotateStates.TRANSFER_HANG, TransferGrabStates.TRANSFER_ADJUST, TransferGimbleStates.GIMBLE_HANG);
        controlTransfer(TransferRotateStates.TRANSFER_HANG, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.GIMBLE_HANG);
        TgrabServoe = TransferGrabStates.TRANSFER_CLOSE;
        TrotateServoe = TransferRotateStates.TRANSFER_HANG;
        TgimbleServoe = TransferGimbleStates.GIMBLE_HANG;
    }

    public void finishHang() {
        verticalLeft1.setTargetPosition(880);
        verticalRight1.setTargetPosition(-880);
        controlTransfer(TransferRotateStates.TRANSFER_HANG, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.GIMBLE_HANG);
        sleep(400);
        // //3. Let go of specimen
        controlTransfer(TransferRotateStates.TRANSFER_HANG, TransferGrabStates.TRANSFER_OPEN, TransferGimbleStates.GIMBLE_HANG);
        // //4. Retract vertical while fliiping transfer to up position
        verticalLeft1.setTargetPosition(500);
        verticalRight1.setTargetPosition(-500);
        controlTransfer(TransferRotateStates.TRANSFER_UP, TransferGrabStates.TRANSFER_OPEN, TransferGimbleStates.TRANSFER_CENTER);
        sleep(400);
        verticalLeft1.setTargetPosition(0);
        verticalRight1.setTargetPosition(0);
        TgrabServoe = TransferGrabStates.TRANSFER_OPEN;
        TrotateServoe = TransferRotateStates.TRANSFER_UP;
        TgimbleServoe = TransferGimbleStates.TRANSFER_CENTER;
    }
}


