package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "TransferFinal")
public class TransferFinal extends LinearOpMode {
    //Defining Servos with Hardware Maps:
    //Ascent Servos
    Servo servostop2;
    Servo servostop1;
    
    //Intake Servos
    Servo rotateServo;
    Servo grabServo;
    Servo gimbleServo;

    //Transfer Servos
    Servo TrotateServo;
    Servo TgrabServo;
    Servo TgimbleServo;
    
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;

    public double left = 0;
    public double right = 0;
    public double drive = 0;
    public double turn = 0;
    public double max = 0;

    //Setting Gamepad Controls to false:
    boolean rightBumper = false;
    boolean leftBumper = false;
    boolean dpadUp = false;

    //Vertical Slide Motors
    public DcMotor verticalLeft;
    public DcMotor verticalRight;

    //Enums:
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
        GIMBLE_HANG
    }

    //Enum Initilization:
    IntakeRotateStates rotateServoe = IntakeRotateStates.INTAKE_UP;
    IntakeGrabStates grabServoe = IntakeGrabStates.GRAB_OPEN;
    IntakeGimbleStates gimbleServoe = IntakeGimbleStates.GIMBLE_NINETY;
    TransferGrabStates TgrabServoe = TransferGrabStates.TRANSFER_CLOSE;
    TransferGimbleStates TgimbleServoe = TransferGimbleStates.GIMBLE_HANG;
    TransferRotateStates TrotateServoe = TransferRotateStates.TRANSFER_HANG;


    @Override
    public void runOpMode(){
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        servoInitializationAuto(hardwareMap);

        waitForStart();

        verticalLeft.setTargetPosition(0);
        verticalRight.setTargetPosition(0);

        verticalLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        verticalLeft.setPower(0.7);
        verticalRight.setPower(0.7);

        while (opModeIsActive()) {

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

            telemetry.addData("drive %f", drive);
            telemetry.addData("turn %f", turn);
            telemetry.update();

            // Gamepad Button Simplified:
            leftBumper = gamepad1.left_bumper;
            rightBumper = gamepad1.right_bumper;
            dpadUp = gamepad1.dpad_up;

            telemetry.addData("vertical left %f", verticalLeft.getCurrentPosition());
            telemetry.addData("vertical right %f", verticalRight.getCurrentPosition());
            telemetry.update();

            // //Buisness Logic for Intake servos are in LM2 TELEOP and for Transfer Servos its not really needed

            // // Business Logic for Transfer Servos:
            //     // Transfer Rotate Servo:
            //     if (TrotateServoe == TransferRotateStates.TRANSFER_UP && gamepad1.right_bumper == true && rightBumper == false) {
            //         TrotateServoe = TransferRotateStates.TRANSFER_MIDDLE;
            //     } else if (TrotateServoe == TransferRotateStates.TRANSFER_MIDDLE && gamepad1.right_bumper == true && rightBumper == false) {
            //         TrotateServoe = TransferRotateStates.TRANSFER_DOWN;
            //     } else if (TrotateServoe == TransferRotateStates.TRANSFER_DOWN && gamepad1.right_bumper == true && rightBumper == false) {
            //         TrotateServoe = TransferRotateStates.TRANSFER_UP;
            //     }

            //     // Transfer Grab Servo:
            //     if (TgrabServoe == TransferGrabStates.TRANSFER_OPEN && gamepad1.left_bumper == true && leftBumper == false) {
            //         TgrabServoe = TransferGrabStates.TRANSFER_CLOSE;
            //     }
            //     else if (TgrabServoe == TransferGrabStates.TRANSFER_CLOSE && gamepad1.left_bumper == true && leftBumper == false) {
            //         TgrabServoe = TransferGrabStates.TRANSFER_OPEN;
            //     }

            //     //Transfer Gimble Servo:
            //     if (TgimbleServoe == TransferGimbleStates.TRANSFER_CENTER && gamepad1.dpad_up == true && dpadUp == false) {
            //         TgimbleServoe = TransferGimbleStates.TRANSFER_NINETY;
            //     }
            //     else if (TgimbleServoe == TransferGimbleStates.TRANSFER_NINETY && gamepad1.dpad_up == true && dpadUp == false) {
            //         TgimbleServoe = TransferGimbleStates.TRANSFER_CENTER;
            //     }

            // // Buisness Logic for Intake Servos:
            //     // Intake Rotate Servo:
            //     if (rotateServoe == IntakeRotateStates.INTAKE_UP && gamepad1.right_bumper == true && rightBumper == false) {
            //     rotateServoe = IntakeRotateStates.INTAKE_MIDDLE;
            //     }
            //     else if (rotateServoe == IntakeRotateStates.INTAKE_MIDDLE && gamepad1.right_bumper == true && rightBumper == false) {
            //     rotateServoe = IntakeRotateStates.INTAKE_DOWN;
            //     }
            //     else if (rotateServoe == IntakeRotateStates.INTAKE_DOWN && gamepad1.right_bumper == true && rightBumper == false) {
            //     rotateServoe = IntakeRotateStates.INTAKE_UP;
            //     }

            //     // Intake Grab Servo:
            //         if (grabServoe == IntakeGrabStates.GRAB_OPEN && gamepad1.left_bumper == true && leftBumper == false) {
            //         grabServoe = IntakeGrabStates.GRAB_CLOSE;
            //     }
            //         else if (grabServoe == IntakeGrabStates.GRAB_CLOSE && gamepad1.left_bumper == true && leftBumper == false) {
            //         grabServoe = IntakeGrabStates.GRAB_OPEN;
            //     }

            //     //Intake Gimble Servo:
            //         if (gimbleServoe == IntakeGimbleStates.GIMBLE_CENTER && gamepad1.dpad_up == true && dpadUp == false) {
            //     gimbleServoe = IntakeGimbleStates.GIMBLE_NINETY;
            //     }
            //         else if (gimbleServoe == IntakeGimbleStates.GIMBLE_NINETY && gamepad1.dpad_up == true && dpadUp == false) {
            //     gimbleServoe = IntakeGimbleStates.GIMBLE_CENTER;
            //     }

            //Buisness Logic for the Macros
            if (gamepad1.dpad_up == true) {
                sampleTransfer();
            }
            if (gamepad1.dpad_down == true) {
                specimenPickup();
            }
            if (gamepad1.dpad_right == true) {
                hangSpecimen();
            }
            if (gamepad1.dpad_left == true) {
                finishHang();
            }

            showTelemetry();

        }
    }


    public void showTelemetry() {
        telemetry.addData("RotateServoPOS", rotateServo.getPosition());
        telemetry.addData("GimbleServoPOS", gimbleServo.getPosition());
        telemetry.addData("GrabServoPOS", grabServo.getPosition());
        telemetry.addData("TransferRotateServoPOS", TrotateServo.getPosition());
        telemetry.addData("TransferGimbleServoPOS", TgimbleServo.getPosition());
        telemetry.addData("TransferGrabServoPOS", TgrabServo.getPosition());
        telemetry.update();
    }
    public void specimenPickup() {
        // 1. Initialization
        //controlTransfer(TransferRotateStates.TRANSFER_DOWN, TransferGrabStates.TRANSFER_OPEN, TransferGimbleStates.TRANSFER_NINETY);
        // 2. Orient the transfer into the correct position
        controlTransfer(TransferRotateStates.TRANSFER_UP, TransferGrabStates.TRANSFER_OPEN, TransferGimbleStates.TRANSFER_CENTER);
        // 2. Grab the specimen
        controlTransfer(TransferRotateStates.TRANSFER_UP, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.TRANSFER_CENTER);
        verticalLeft.setTargetPosition(300);
        verticalRight.setTargetPosition(-300);
        sleep(400);
        // 3. Rotate the transfer back
        // controlTransfer(TransferRotateStates.TRANSFER_DOWN, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.TRANSFER_CENTER);
    }

    public void hangSpecimen() {
        //1. Intialize transfer in the up position
        //this.controlTransfer(TransferRotateStates.TRANSFER_UP, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.TRANSFER_CENTER);
        //2. Raise vertical while transfer rotates into down position
        this.controlTransfer(TransferRotateStates.TRANSFER_HANG, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.GIMBLE_HANG);
        this.controlTransfer(TransferRotateStates.TRANSFER_HANG, TransferGrabStates.TRANSFER_ADJUST, TransferGimbleStates.GIMBLE_HANG);
        this.controlTransfer(TransferRotateStates.TRANSFER_HANG, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.GIMBLE_HANG);

    }

    public void finishHang() {
        this.verticalLeft.setTargetPosition(890);
        this.verticalRight.setTargetPosition(-890);
        this.controlTransfer(TransferRotateStates.TRANSFER_HANG, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.GIMBLE_HANG);
        sleep(400);
        // //3. Let go of specimen
        this.controlTransfer(TransferRotateStates.TRANSFER_HANG, TransferGrabStates.TRANSFER_OPEN, TransferGimbleStates.GIMBLE_HANG);
        // //4. Retract vertical while fliiping transfer to up position
        this.verticalLeft.setTargetPosition(500);
        this.verticalRight.setTargetPosition(-500);
        this.controlTransfer(TransferRotateStates.TRANSFER_UP, TransferGrabStates.TRANSFER_OPEN, TransferGimbleStates.TRANSFER_CENTER);
        sleep(400);
        this.verticalLeft.setTargetPosition(0);
        this.verticalRight.setTargetPosition(0);
    }
    public void sampleTransfer() {
        // 1. Position the intake claw right above the sample
        // controlIntake(IntakeRotateStates.INTAKE_MIDDLE, IntakeGrabStates.GRAB_OPEN, IntakeGimbleStates.GIMBLE_NINETY);
        // //sleep(250);
        // // 2. Pickup the sample with the intake claw
        // controlIntake(IntakeRotateStates.INTAKE_DOWN, IntakeGrabStates.GRAB_CLOSE, IntakeGimbleStates.GIMBLE_NINETY);
        //sleep(250);
        // 3. Position the rotate servo to the correct position
        this.controlIntake(IntakeRotateStates.INTAKE_UP, IntakeGrabStates.GRAB_CLOSE, IntakeGimbleStates.GIMBLE_NINETY);
        // sleep(250);
        // 4. Loose power to position the sample correctly in the claw
        this.controlIntake(IntakeRotateStates.INTAKE_UP, IntakeGrabStates.GRAB_ADJUST, IntakeGimbleStates.GIMBLE_NINETY);
        //sleep(250);
        // 5. Bring the transfer directly over the sample
        this.controlTransfer(TransferRotateStates.TRANSFER_DOWN, TransferGrabStates.TRANSFER_OPEN, TransferGimbleStates.TRANSFER_NINETY);
        sleep(1000);
        // 6. Grab the sample with the transfer claw
        this.controlTransfer(TransferRotateStates.TRANSFER_DOWN, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.TRANSFER_NINETY);
        //sleep(250);
        // 7. Rotate the transfer while holding the sample
        this.controlTransfer(TransferRotateStates.TRANSFER_UP, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.TRANSFER_NINETY);
    }
    public void controlIntake(IntakeRotateStates rotate, IntakeGrabStates grab, IntakeGimbleStates gimble) {
        //Hardware Calls for Intake Servos:
        //Intake Rotate Servo:
        if (rotate == IntakeRotateStates.INTAKE_DOWN)
            this.rotateServo.setPosition(0.15);//rotating intake fully down
        if (rotate == IntakeRotateStates.INTAKE_UP)
            this.rotateServo.setPosition(0.95);//rotating intake fully upright
        if (rotate == IntakeRotateStates.INTAKE_MIDDLE)
            this.rotateServo.setPosition(0.35);//rotate intake middle position
        sleep(300);

        //Intake Grab Servo:
        if (grab == IntakeGrabStates.GRAB_CLOSE)
            this.grabServo.setPosition(1);//close claw fully

        if (grab == IntakeGrabStates.GRAB_OPEN)
            this.grabServo.setPosition(0);//open claw fully

        if (grab == IntakeGrabStates.GRAB_ADJUST) {
            this.grabServo.setPosition(0.5);
            sleep(300);
            this.grabServo.setPosition(1);
        }
        sleep(150);

        //Intake Gimble Servo:
        if (gimble == IntakeGimbleStates.GIMBLE_CENTER)
            this.gimbleServo.setPosition(0.55);

        if (gimble == IntakeGimbleStates.GIMBLE_NINETY)
            this.gimbleServo.setPosition(1);
        sleep(250);
    }
    public void controlTransfer(TransferRotateStates rotate, TransferGrabStates grab, TransferGimbleStates gimble) {
        // Hardware Calls for Transfer Servos:
        // Transfer Rotate Servo:
        if (rotate == TransferRotateStates.TRANSFER_DOWN)
            this.TrotateServo.setPosition(1);//rotating intake fully down
        if (rotate == TransferRotateStates.TRANSFER_UP)
            this.TrotateServo.setPosition(0);//rotating intake fully upright
        if (rotate == TransferRotateStates.TRANSFER_MIDDLE)
            this.TrotateServo.setPosition(0.3);//rotate intake middle position
        if (rotate == TransferRotateStates.TRANSFER_HANG)
            this.TrotateServo.setPosition(0.625);//rotate intake middle position
        sleep(250);

        //Transfer Grab Servo:
        if (grab == TransferGrabStates.TRANSFER_CLOSE) {
            this.TgrabServo.setPosition(0); //Close Claw
            sleep(100);
            this.grabServo.setPosition(0);
        }
        if (grab == TransferGrabStates.TRANSFER_OPEN){
            this.TgrabServo.setPosition(1);//Open Claw
        }
        if (grab == TransferGrabStates.TRANSFER_ADJUST){
            this.TgrabServo.setPosition(0.4);//Open Claw
        }
        sleep(250);

        //Transfer Gimble Servo:
        if (gimble == TransferGimbleStates.TRANSFER_CENTER){
            this.TgimbleServo.setPosition(0.7);
        }
        if (gimble == TransferGimbleStates.TRANSFER_NINETY){
            this.TgimbleServo.setPosition(1);
        }
        if (gimble == TransferGimbleStates.GIMBLE_HANG){
            this.TgimbleServo.setPosition(0.8);
        }
        sleep(250);

    }

    public void servoInitializationAuto(HardwareMap hmap) {
        //Defining Servos with Hardware Maps:
        //Ascent Servos
        this.servostop2 = hmap.get(Servo.class, "cs1");
        this.servostop1 = hmap.get(Servo.class, "cs3");

        //Intake Servos
        this.rotateServo = hmap.get(Servo.class, "es3");
        this.grabServo = hmap.get(Servo.class, "es5");
        this.gimbleServo = hmap.get(Servo.class, "es1");

        //Transfer Servos
        this.TrotateServo = hmap.get(Servo.class, "cs5");
        this.TgrabServo = hmap.get(Servo.class, "cs2");
        this.TgimbleServo = hmap.get(Servo.class, "cs4");

        this.verticalLeft = hmap.get(DcMotor.class, "em1");
        this.verticalRight = hmap.get(DcMotor.class, "em2");

        this.rotateServo.setPosition(0.95);
        this.gimbleServo.setPosition(1.0);
        this.grabServo.setPosition(0.0);
        this.TrotateServo.setPosition(0.625);
        this.TgimbleServo.setPosition(0.8);
        this.TgrabServo.setPosition(0.0);

        this.servostop2.setPosition(0.20);
        this.servostop1.setPosition(1.0);




//        this.verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        this.verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        this.verticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.verticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
