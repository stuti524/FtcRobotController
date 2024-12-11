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

@TeleOp(name = "LM2Teleop", group = "TeleOp")
public class LM2Teleop extends LinearOpMode {
    enum StateE {
        SAMPLE_STATE,
        CHAMBER_STATE,
        WALL_STATE,
        ASCENT_STATE
    }

    ;

    enum VertE {
        ABOVE_CHAM,
        ON_CHAM,
        RESET_CHAM
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


    enum RotateServoE {
        INTAKE_DOWN,
        INTAKE_MIDDLE,
        INTAKE_UP
    }

    enum GrabServoE {
        GRAB_CLOSE,
        GRAB_OPEN
    }

    enum GimbleServoE {
        GIMBLE_CENTER,
        GIMBLE_NINETY
    }

    //SERVO CODE
    // Servo rotateServo;

    //clawServo
    Servo clawservo;
    ClawServoE clawservoe = ClawServoE.CLAW_CLOSE;
    VertE vertchame = VertE.RESET_CHAM;
    //Wall

    enum ClawServoE {
        //INTAKE_NO_DOWN_UP,
        CLAW_CLOSE,
        CLAW_OPEN
    }

    ;

    Servo clawservo1;
    ClawServoE clawservoe1 = ClawServoE.CLAW_CLOSE;

    enum HangE {
        ABOVE_RUNG,
        ON_RUNG,
        ZERO_RUNG
    }

    HangE hangE = HangE.ZERO_RUNG;
    Servo servostop1;
    Servo servostop2;

    //SERVO CODE
    Servo rotateServo;
    Servo grabServo;
    Servo gimbleServo;

    RotateServoE rotateServoe = RotateServoE.INTAKE_UP;
    GrabServoE grabServoe = GrabServoE.GRAB_CLOSE;
    GimbleServoE gimbleServoe = GimbleServoE.GIMBLE_CENTER;

    boolean rightBumper = false;
    boolean leftBumper = false;
    boolean dpadUp = false;

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

        //Sample

        //VERTICAL

        horizontalMotor1 = hardwareMap.get(DcMotor.class, "em0");
        horizontalMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalMotor1.setTargetPosition(0);
        horizontalMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalMotor1.setPower(1);

        rotateServo = hardwareMap.get(Servo.class, "es3");

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

        verticalLeft1.setPower(0.7);
        verticalRight1.setPower(0.7);


        while (opModeIsActive()) {
            if (state != StateE.SAMPLE_STATE) {
                rotateServo.setPosition(0.87);//rotating intake fully upright
                grabServo.setPosition(1);
                gimbleServo.setPosition(0.55);
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
        clawservo1.setPosition(0);
        if (GPRB && hangE == HangE.ZERO_RUNG) {
            hangE = HangE.ABOVE_RUNG;
        }
        //Hardware calls
        if (hangE == HangE.ABOVE_RUNG) {
            verticalLeft1.setTargetPosition(1500);
            verticalRight1.setTargetPosition(-1500);
            //THESE SERVO POS WORK
            servostop1.setPosition(0);
            servostop2.setPosition(1.0);
        }

        telemetry.addData("vertical left %f", verticalLeft1.getCurrentPosition());
        telemetry.addData("vertical right %f", verticalRight1.getCurrentPosition());
        telemetry.update();
    }

    public void ascentFinal() {
        boolean GPRB = gamepad1.right_bumper;
        if (GPRB && hangE == HangE.ABOVE_RUNG) {
            hangE = HangE.ON_RUNG;
        }
        if (hangE == HangE.ON_RUNG) {
            verticalLeft1.setTargetPosition(0);
            verticalRight1.setTargetPosition(0);
        }
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

        telemetry.addData("drive %f", drive);
        telemetry.addData("turn %f", turn);
        telemetry.update();

    }

    public void sample() {

        //Business with Hardware
        if (rightArrow == false && gamepad1.dpad_right == true) {
            rightArrow = true;
        }
        if (rightArrow == true && gamepad1.dpad_right == false) {
            rightArrow = false;
        }
        if (leftArrow == false && gamepad1.dpad_left == true) {
            leftArrow = true;

        }
        if (leftArrow == true && gamepad1.dpad_left == false) {
            leftArrow = false;
        }
        if (rightArrow == true) {
            horizontalMotor1.setTargetPosition(-3000);
        }
        if (leftArrow == true) {
            horizontalMotor1.setTargetPosition(0);
        }
        //CLAW INTAKE CODE

        //ROTATE SERVO
        //MACRO
        if (gamepad1.dpad_down == true) {
            grabServoe = GrabServoE.GRAB_OPEN;
            sleep(250);
            rotateServoe = RotateServoE.INTAKE_DOWN;
            rotateServo.setPosition(0.25);
            sleep(250);
            grabServoe = GrabServoE.GRAB_CLOSE;
            grabServo.setPosition(1);//close claw fully
            sleep(250);
            rotateServoe = RotateServoE.INTAKE_UP;
            rotateServo.setPosition(0.87);//rotating intake fully upright
            sleep(500);
        }
        //business logic
        if (rotateServoe == RotateServoE.INTAKE_UP && gamepad1.right_bumper == true && rightBumper == false) {
            rotateServoe = RotateServoE.INTAKE_MIDDLE;
        } else if (rotateServoe == RotateServoE.INTAKE_MIDDLE && gamepad1.right_bumper == true && rightBumper == false) {
            rotateServoe = RotateServoE.INTAKE_DOWN;
        } else if (rotateServoe == RotateServoE.INTAKE_DOWN && gamepad1.right_bumper == true && rightBumper == false) {
            rotateServoe = RotateServoE.INTAKE_UP;
        }

        rightBumper = gamepad1.right_bumper;

        // hardware calls
        if (rotateServoe == RotateServoE.INTAKE_DOWN)
            rotateServo.setPosition(0.25);//rotating intake fully down
        if (rotateServoe == RotateServoE.INTAKE_UP)
            rotateServo.setPosition(0.87);//rotating intake fully upright
        if (rotateServoe == RotateServoE.INTAKE_MIDDLE)
            rotateServo.setPosition(0.35);//rotate intake middle position
        telemetry.addData("RotateServoPOS", rotateServo.getPosition());
        telemetry.update();


        //GRAB SERVO

        //business logic
        if (grabServoe == GrabServoE.GRAB_OPEN && gamepad1.left_bumper == true && leftBumper == false) {
            grabServoe = GrabServoE.GRAB_CLOSE;
        } else if (grabServoe == GrabServoE.GRAB_CLOSE && gamepad1.left_bumper == true && leftBumper == false) {
            grabServoe = GrabServoE.GRAB_OPEN;
        }
        leftBumper = gamepad1.left_bumper;

        //hardware calls
        if (grabServoe == GrabServoE.GRAB_CLOSE)
            grabServo.setPosition(1);//close claw fully

        if (grabServoe == GrabServoE.GRAB_OPEN)
            grabServo.setPosition(0);//open claw fully


        telemetry.addData("GrabServoPOS", grabServo.getPosition());

        //GIMBLE SERVO

        //business logic
        if (gimbleServoe == GimbleServoE.GIMBLE_CENTER && gamepad1.dpad_up == true && dpadUp == false) {
            gimbleServoe = GimbleServoE.GIMBLE_NINETY;
        } else if (gimbleServoe == GimbleServoE.GIMBLE_NINETY && gamepad1.dpad_up == true && dpadUp == false) {
            gimbleServoe = GimbleServoE.GIMBLE_CENTER;
        }

        dpadUp = gamepad1.dpad_up;

        // hardware calls
        if (gimbleServoe == GimbleServoE.GIMBLE_CENTER)
            gimbleServo.setPosition(0.55);
        if (gimbleServoe == GimbleServoE.GIMBLE_NINETY)
            gimbleServo.setPosition(0.9);
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
//        boolean gamepaddpad_down1 = gamepad1.dpad_down;
//        boolean gamepaddpad_right1 = gamepad1.dpad_right;

        // business logic
        if (vertchame == VertE.RESET_CHAM && gamepaddpad_up1 == true) { // retracted and gamepad.y is held
            vertchame = VertE.ABOVE_CHAM;
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
        if (gamepad1.dpad_down == true) {
            vertchame = VertE.ON_CHAM;
            verticalLeft1.setTargetPosition(900);
            verticalRight1.setTargetPosition(-900);
            sleep(700);
            clawservoe1 = ClawServoE.CLAW_OPEN;
            clawservo1.setPosition(0); // vertical extension claw is open
            sleep(250);
            vertchame = VertE.RESET_CHAM;
            verticalLeft1.setTargetPosition(0);
            verticalRight1.setTargetPosition(0);
            sleep(1500);
        }
        //}

        // hardware calls
        if (vertchame == VertE.ABOVE_CHAM) {
            verticalLeft1.setTargetPosition(1300);
            verticalRight1.setTargetPosition(-1300);
        } else if (vertchame == VertE.ON_CHAM) {
            verticalLeft1.setTargetPosition(900);
            verticalRight1.setTargetPosition(-900);
        } else if (vertchame == VertE.RESET_CHAM) {
            verticalLeft1.setTargetPosition(0);
            verticalRight1.setTargetPosition(0);
        }
        telemetry.addData("ClawServoPOS!", clawservo1.getPosition());

        telemetry.update();

        telemetry.addData("Vertical Left Motor Position", verticalLeft1.getCurrentPosition());
        telemetry.addData("Vertical Right Motor Position", verticalRight1.getCurrentPosition());

        telemetry.update();
    }
}

