package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "LM3Teleop")
public class LM3Teleop extends LinearOpMode {
    volatile double driveSpeedFactor = 0.67;
    TransferFinal tf = new TransferFinal();

    enum StateE {
        SAMPLE_STATE,
        CHAMBER_STATE,
        WALL_STATE,
        ASCENT_STATE
    }

    enum VertE {
        ABOVE_BASK,
        ZERO_BASK,
        NULL_BASK

    }

    enum HortE {
        HORI_EXTEND,
        HORI_RETRACT
    }

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
    public DcMotor horizontalMotor1 = null;
    public DcMotor verticalLeft1 = null;
    public DcMotor verticalRight1 = null;
    public boolean rightArrow = false;

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

    VertE vertbaske = VertE.ZERO_BASK;

    HortE hortE = HortE.HORI_RETRACT;

    enum HangE {
        ABOVE_RUNG,
        ON_RUNG,
        ZERO_RUNG,
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
    DigitalChannel digitalHori;
    DigitalChannel digitalVert;

    //Enum Initilization:
    IntakeRotateStates rotateServoe = IntakeRotateStates.INTAKE_UP;
    IntakeGrabStates grabServoe = IntakeGrabStates.GRAB_OPEN;
    IntakeGimbleStates gimbleServoe = IntakeGimbleStates.GIMBLE_NINETY;
    TransferGrabStates TgrabServoe = TransferGrabStates.TRANSFER_CLOSE;
    TransferGimbleStates TgimbleServoe = TransferGimbleStates.TRANSFER_CENTER;
    TransferRotateStates TrotateServoe = TransferRotateStates.TRANSFER_UP;

    boolean rightBumper = false;
    boolean leftBumper = false;
    boolean dpadUp = false;
    boolean upArrow = false;
    final boolean PRESSED = false;

    @Override

    public void runOpMode() {

        digitalHori = hardwareMap.get(DigitalChannel.class, "cd7");
        digitalHori.setMode(DigitalChannel.Mode.INPUT);

        digitalVert = hardwareMap.get(DigitalChannel.class, "ed7");
        digitalVert.setMode(DigitalChannel.Mode.INPUT);

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

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        tf.servoInitializationTeleop(hardwareMap, 1);

        horizontalMotor1 = hardwareMap.get(DcMotor.class, "em0");
        horizontalMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalMotor1.setTargetPosition(0);
        horizontalMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalMotor1.setPower(1);

//Transfer Servos
        TrotateServo = hardwareMap.get(Servo.class, "cs5");
        TgrabServo = hardwareMap.get(Servo.class, "cs2");
        TgimbleServo = hardwareMap.get(Servo.class, "cs4");

        verticalLeft1 = hardwareMap.get(DcMotor.class, "em1");
        verticalRight1 = hardwareMap.get(DcMotor.class, "em2");

        verticalLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ((DcMotorEx) verticalLeft1).setTargetPositionTolerance(10);
        ((DcMotorEx) verticalRight1).setTargetPositionTolerance(10);
        ((DcMotorEx) horizontalMotor1).setTargetPositionTolerance(30);

        rotateServo.setPosition(0.95);//rotating intake fully upright
        grabServo.setPosition(1);
        gimbleServo.setPosition(0.275);

        while (digitalHori.getState() != PRESSED) {
            horizontalMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            horizontalMotor1.setPower(0.4);

        }

        while (digitalVert.getState() != PRESSED) {
            verticalRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            verticalLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            verticalRight1.setPower(0.4);
            verticalLeft1.setPower(-0.4);

        }

        resetVerticalHorizontal();


        waitForStart();

        rotateServo.setPosition(0.95);//rotating intake fully upright
        grabServo.setPosition(1);
        gimbleServo.setPosition(0.275);

        verticalLeft1.setTargetPosition(0);
        verticalRight1.setTargetPosition(0);

        verticalLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        verticalLeft1.setPower(1);
        verticalRight1.setPower(1);

        while (opModeInInit()) {
        }

        Thread driveThread = new Thread(()
                -> {
            while (opModeIsActive()) {
                drive();
            }
        });
        driveThread.start();

        telemetry.setMsTransmissionInterval(0);

        while (opModeIsActive()) {
            if (state != StateE.SAMPLE_STATE) {
                rotateServo.setPosition(0.95);//rotating intake fully upright
                grabServo.setPosition(1);
                gimbleServo.setPosition(0.275);
            }

            if (state != StateE.ASCENT_STATE) {
                servostop1.setPosition(1.0);
                servostop2.setPosition(0.09);
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
                sample();
            }
            if (state == StateE.CHAMBER_STATE) {
                chamber();
            }

            if (state == StateE.ASCENT_STATE) {
                ascent();
                while (gamepad1.left_bumper) {
                    ascentFinal();
                }
            }

            double rf = ((DcMotorEx) rightFrontDrive).getCurrent(CurrentUnit.MILLIAMPS);
            double rb = ((DcMotorEx) rightBackDrive).getCurrent(CurrentUnit.MILLIAMPS);
            double lf = ((DcMotorEx) leftFrontDrive).getCurrent(CurrentUnit.MILLIAMPS);
            double lb = ((DcMotorEx) leftBackDrive).getCurrent(CurrentUnit.MILLIAMPS);
            double vl1 = ((DcMotorEx) verticalLeft1).getCurrent(CurrentUnit.MILLIAMPS);
            double vr1 = ((DcMotorEx) verticalRight1).getCurrent(CurrentUnit.MILLIAMPS);
            double hm1 = ((DcMotorEx) horizontalMotor1).getCurrent(CurrentUnit.MILLIAMPS);
            telemetry.addLine(String.format("VL: %.1f\nVR: %.1f\nH: %.1f", vl1 / 1000, vr1 / 1000, hm1 / 1000));
            telemetry.addLine(String.format("Rf: %.1f, Rb: %.1f, Lf: %.1f, Lb: %.1f", rf / 1000, rb / 1000, lf / 1000, lb / 1000));
            telemetry.update();
        }
    }

    public void ascent() {

        resetVerticalHorizontal();

        boolean GPRB = gamepad1.right_bumper;
        if (GPRB && hangE == HangE.ZERO_RUNG) {
            hangE = HangE.ABOVE_RUNG;
        }
        //Hardware calls
        if (hangE == HangE.ABOVE_RUNG) {
            verticalLeft1.setPower(1);
            verticalRight1.setPower(1);
            verticalLeft1.setTargetPosition(2925);
            verticalRight1.setTargetPosition(-2925);
            TrotateServo.setPosition(0.55);
            TgimbleServo.setPosition(1);
            sleep(200);
            //THESE SERVO POS WORK
            servostop1.setPosition(0.10);
            servostop2.setPosition(0.60);
        }

    }

    public void ascentFinal() {
        boolean GPRB = gamepad1.right_bumper;
        horizontalMotor1.setPower(0);

        //Business Logic
        if (GPRB && hangE == HangE.ABOVE_RUNG) {
            hangE = HangE.ON_RUNG;
        }

        //L2
        if (hangE == HangE.ON_RUNG) {
            double vr = verticalRight1.getCurrentPosition();
            double vl = verticalLeft1.getCurrentPosition();
            grabServo.close();
            rotateServo.close();
            gimbleServo.close();
            TrotateServo.close();
            TgrabServo.close();
            TgimbleServo.close();
            servostop1.close();
            servostop2.close();
            verticalLeft1.setPower(1);
            verticalRight1.setPower(1);
            verticalLeft1.setTargetPosition(0);
            verticalRight1.setTargetPosition(0);
            sleep(500);
        }
    }

    public void drive() {
        driveSpeedFactor = 0.67;
        float leftTrigger = gamepad1.left_trigger;
        if (leftTrigger > 0) {
            driveSpeedFactor = 0.2;
        }
        //Drive
        // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        // This way it's also easy to just drive straight, or just turn.
        drive = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;

        // Combine drive and turn for blended motion.
        //TEMP SOL - DIV by 2 to slow down the DT
        left = (drive + turn) * driveSpeedFactor;
        right = (drive - turn) * driveSpeedFactor;

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
    }

    public void sample() {

        resetVerticalHorizontal();

        if (gamepad1.dpad_left) {
            hortE = HortE.HORI_RETRACT;
            horizontalMotor1.setTargetPosition(0);
            sleep(300);
            tf.sampleTransfer();
        }
        if (TrotateServoe == TransferRotateStates.TRANSFER_UP) {
            TgimbleServoe = TransferGimbleStates.TRANSFER_NINETY;
            TgimbleServo.setPosition(0.9);
        }

        if (rightArrow == false && gamepad1.dpad_right == true && hortE == HortE.HORI_RETRACT) {
            hortE = HortE.HORI_EXTEND;
        } else if (rightArrow == false && gamepad1.dpad_right == true && hortE == HortE.HORI_EXTEND) {
            hortE = HortE.HORI_RETRACT;
            ;
        }
        rightArrow = gamepad1.dpad_right;

        if (hortE == HortE.HORI_EXTEND) {
            horizontalMotor1.setPower(1);
            horizontalMotor1.setTargetPosition(-2900);
        }
        if (hortE == HortE.HORI_RETRACT) {
            horizontalMotor1.setTargetPosition(0);
            horizontalMotor1.setPower(-0.7);
        }

        //CLAW INTAKE CODE

        //MACRO
        if (gamepad1.dpad_down) {
            grabServoe = IntakeGrabStates.GRAB_OPEN;
            grabServo.setPosition(0);
            sleep(100);
            rotateServoe = IntakeRotateStates.INTAKE_DOWN;
            rotateServo.setPosition(0.1);
            sleep(200);
            grabServoe = IntakeGrabStates.GRAB_CLOSE;
            grabServo.setPosition(0.56);
            sleep(100);
            rotateServoe = IntakeRotateStates.INTAKE_UP;
            rotateServo.setPosition(0.95);//rotating intake fully upright
            sleep(200);
        }

        //ROTATE SERVO

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
            rotateServo.setPosition(0.95);//rotating intake fully upright
        if (rotateServoe == IntakeRotateStates.INTAKE_MIDDLE)
            rotateServo.setPosition(0.25);//rotate intake middle position

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
            grabServo.setPosition(0.56);//close claw fully

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
        if (gimbleServoe == IntakeGimbleStates.GIMBLE_CENTER) {
            gimbleServo.setPosition(0.625);
        }
        if (gimbleServoe == IntakeGimbleStates.GIMBLE_NINETY) {
            gimbleServo.setPosition(1.0);
        }
        TgrabServoe = TransferGrabStates.TRANSFER_CLOSE;
        TgimbleServoe = TransferGimbleStates.TRANSFER_CENTER;
    }

    public void controlIntake(IntakeRotateStates rotate, IntakeGrabStates grab, IntakeGimbleStates gimble) {

        //Hardware Calls for Intake Servos:

        //Intake Rotate Servo:

        if (rotate == IntakeRotateStates.INTAKE_DOWN)
            rotateServo.setPosition(0.1);//rotating intake fully down
        if (rotate == IntakeRotateStates.INTAKE_UP)
            rotateServo.setPosition(0.95);//rotating intake fully upright
        if (rotate == IntakeRotateStates.INTAKE_MIDDLE)
            rotateServo.setPosition(0.35);//rotate intake middle position
        sleep(150);

        //Intake Grab Servo:
        if (grab == IntakeGrabStates.GRAB_CLOSE)
            grabServo.setPosition(0.56);//close claw fully

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
            gimbleServo.setPosition(0.625);

        if (gimble == IntakeGimbleStates.GIMBLE_NINETY)
            gimbleServo.setPosition(1);
        sleep(150);
    }

    public void controlTransfer(TransferRotateStates rotate, TransferGrabStates grab, TransferGimbleStates gimble) {
        // Hardware Calls for Transfer Servos:
        // Transfer Rotate Servo:
        if (rotate == TransferRotateStates.TRANSFER_DOWN)
            TrotateServo.setPosition(0.72);//rotating intake fully down
        if (rotate == TransferRotateStates.TRANSFER_UP)
            TrotateServo.setPosition(0.08);//rotating intake fully upright
        if (rotate == TransferRotateStates.TRANSFER_MIDDLE)
            TrotateServo.setPosition(0.25);//rotate intake middle position
        if (rotate == TransferRotateStates.TRANSFER_HANG)
            TrotateServo.setPosition(0.64);//rotate intake middle position
        sleep(150);

        //Transfer Grab Servo:
        if (grab == TransferGrabStates.TRANSFER_CLOSE) {
            TgrabServo.setPosition(0.60); //Close Claw
            sleep(100);
            grabServo.setPosition(0);
        }
        if (grab == TransferGrabStates.TRANSFER_OPEN) {
            TgrabServo.setPosition(0.82);//Open Claw
        }
        if (grab == TransferGrabStates.TRANSFER_ADJUST) {
            TgrabServo.setPosition(0.66);//Open Claw
        }
        sleep(150);

        //Transfer Gimble Servo:
        if (gimble == TransferGimbleStates.TRANSFER_CENTER) {
            TgimbleServo.setPosition(0.46);
        }
        if (gimble == TransferGimbleStates.TRANSFER_NINETY) {
            TgimbleServo.setPosition(0.9);
        }
        if (gimble == TransferGimbleStates.GIMBLE_HANG) {
            TgimbleServo.setPosition(0.78);
        }
        sleep(150);

    }

    public void chamber() {

        resetVerticalHorizontal();

        if ((vertbaske == VertE.ZERO_BASK || vertbaske == VertE.NULL_BASK) && gamepad1.dpad_up == true && upArrow == false) {
            vertbaske = VertE.ABOVE_BASK;
        } else if (vertbaske == VertE.ABOVE_BASK && gamepad1.dpad_up == true && upArrow == false) {
            vertbaske = VertE.ZERO_BASK;
        }
        upArrow = gamepad1.dpad_up;

        if (vertbaske == VertE.ABOVE_BASK) {
            verticalLeft1.setPower(1);
            verticalRight1.setPower(1);
            verticalLeft1.setTargetPosition(4095);
            verticalRight1.setTargetPosition(-4095);
            TrotateServoe = TransferRotateStates.TRANSFER_MIDDLE;
            TrotateServo.setPosition(0.25);//rotate
            TgimbleServoe = TransferGimbleStates.GIMBLE_BASKET;
            TgimbleServo.setPosition(0.54);

        }

        if (vertbaske == VertE.ZERO_BASK) {
            verticalLeft1.setPower(1);
            verticalRight1.setPower(1);
            do {
                verticalLeft1.setTargetPosition(0);
                verticalRight1.setTargetPosition(0);
            }
            while (verticalLeft1.isBusy() || verticalRight1.isBusy());
            verticalLeft1.setPower(0);
            verticalRight1.setPower(0);
            TrotateServoe = TransferRotateStates.TRANSFER_UP;
            TrotateServo.setPosition(0.08);
        }

        if (gamepad1.dpad_left == true) {
            specimenPickup();
            hangSpecimen();
        }
        if (gamepad1.dpad_down == true) {
            finishHang();
        }

        //Business Logic for Transfer Servos:

        //Transfer Rotate Servo:
        if (TrotateServoe == TransferRotateStates.TRANSFER_UP && gamepad1.right_bumper == true && rightBumper == false) {
            TrotateServoe = TransferRotateStates.TRANSFER_MIDDLE;
        } else if (TrotateServoe == TransferRotateStates.TRANSFER_MIDDLE && gamepad1.right_bumper == true && rightBumper == false) {
            TrotateServoe = TransferRotateStates.TRANSFER_DOWN;
        } else if (TrotateServoe == TransferRotateStates.TRANSFER_DOWN && gamepad1.right_bumper == true && rightBumper == false) {
            TrotateServoe = TransferRotateStates.TRANSFER_UP;
        }

        rightBumper = gamepad1.right_bumper;

        //Transfer Grab Servo:
        if (TgrabServoe == TransferGrabStates.TRANSFER_OPEN && gamepad1.left_bumper == true && leftBumper == false) {
            TgrabServoe = TransferGrabStates.TRANSFER_CLOSE;
        } else if (TgrabServoe == TransferGrabStates.TRANSFER_CLOSE && gamepad1.left_bumper == true && leftBumper == false) {
            TgrabServoe = TransferGrabStates.TRANSFER_OPEN;
        }

        leftBumper = gamepad1.left_bumper;

        if (TgrabServoe == TransferGrabStates.TRANSFER_CLOSE) {
            TgrabServo.setPosition(0.60);//close claw fully
        }

        if (TgrabServoe == TransferGrabStates.TRANSFER_OPEN) {
            TgrabServo.setPosition(0.82);//close claw fully
        }

        if (TrotateServoe == TransferRotateStates.TRANSFER_DOWN) {
            TrotateServo.setPosition(0.72);//rotating intake fully down
        }

        if (TrotateServoe == TransferRotateStates.TRANSFER_UP) {
            TrotateServo.setPosition(0.08);//rotating intake fully upright
        }

        if (TrotateServoe == TransferRotateStates.TRANSFER_MIDDLE) {
            TrotateServo.setPosition(0.25);//rotate intake middle position
        }
        if (TgimbleServoe == TransferGimbleStates.TRANSFER_CENTER) {
            TgimbleServo.setPosition(0.46);
        }
        if (TgimbleServoe == TransferGimbleStates.GIMBLE_HANG) {
            TgimbleServo.setPosition(0.78);
        }
    }

    public void specimenPickup() {
        verticalLeft1.setPower(1);
        verticalRight1.setPower(1);
        controlTransfer(TransferRotateStates.TRANSFER_UP, TransferGrabStates.TRANSFER_OPEN, TransferGimbleStates.TRANSFER_CENTER);
        controlTransfer(TransferRotateStates.TRANSFER_UP, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.TRANSFER_CENTER);
        verticalLeft1.setTargetPosition(585);
        verticalRight1.setTargetPosition(-585);
        vertbaske = VertE.NULL_BASK;
        sleep(400);
        TgrabServoe = TransferGrabStates.TRANSFER_CLOSE;
        TrotateServoe = TransferRotateStates.TRANSFER_UP;
        TgimbleServoe = TransferGimbleStates.TRANSFER_CENTER;
    }

    public void hangSpecimen() {
        verticalLeft1.setPower(1);
        verticalRight1.setPower(1);
        this.verticalLeft1.setTargetPosition(2600);
        this.verticalRight1.setTargetPosition(-2600);
        controlTransfer(TransferRotateStates.TRANSFER_UP, TransferGrabStates.TRANSFER_ADJUST, TransferGimbleStates.TRANSFER_CENTER);

        controlTransfer(TransferRotateStates.TRANSFER_HANG, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.GIMBLE_HANG);
//        controlTransfer(TransferRotateStates.TRANSFER_HANG, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.GIMBLE_HANG);
        TgrabServoe = TransferGrabStates.TRANSFER_CLOSE;
        TrotateServoe = TransferRotateStates.TRANSFER_HANG;
        TgimbleServoe = TransferGimbleStates.GIMBLE_HANG;
        vertbaske = VertE.NULL_BASK;
    }

    public void finishHang() {
//        verticalLeft1.setPower(1);
//        verticalRight1.setPower(1);
//        verticalLeft1.setTargetPosition(2200); //move to 960 if vertical is slow or inconsistent//1900
//        verticalRight1.setTargetPosition(-2200); ///move to -960 if vertical is slow or inconsistent
//        while (verticalRight1.isBusy() || verticalLeft1.isBusy()) {
//        }
//        controlTransfer(TransferRotateStates.TRANSFER_HANG, TransferGrabStates.TRANSFER_OPEN, TransferGimbleStates.GIMBLE_HANG);
//        verticalLeft1.setTargetPosition(975);
//        verticalRight1.setTargetPosition(-975);
        controlTransfer(TransferRotateStates.TRANSFER_HANG, TransferGrabStates.TRANSFER_OPEN, TransferGimbleStates.GIMBLE_HANG);
        verticalLeft1.setTargetPosition(0);
        verticalRight1.setTargetPosition(0);
        controlTransfer(TransferRotateStates.TRANSFER_UP, TransferGrabStates.TRANSFER_OPEN, TransferGimbleStates.TRANSFER_CENTER);

        while (verticalLeft1.isBusy() || verticalRight1.isBusy());
        TgrabServoe = TransferGrabStates.TRANSFER_OPEN;
        TrotateServoe = TransferRotateStates.TRANSFER_UP;
        TgimbleServoe = TransferGimbleStates.TRANSFER_CENTER;
        verticalLeft1.setPower(0);
        verticalRight1.setPower(0);
    }

    private boolean horizontalReset = false;
    private boolean verticalReset = false;

    public void resetVerticalHorizontal() {
        // Horizontal motor reset
        if (digitalHori.getState() == PRESSED) {
            if (!horizontalReset) {
                // Reset encoder and set target position to 0
                horizontalMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                horizontalMotor1.setTargetPosition(0);
                horizontalMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizontalMotor1.setPower(0.5); // Set power to move the motor
                horizontalReset = true; // Mark reset as complete
            }
            // Stop motor if it's already at the target position
            if (horizontalMotor1.isBusy()) {
                // Motor is still moving, no action needed
            } else {
                horizontalMotor1.setPower(0); // Stop motor once target is reached
            }
        } else {
            horizontalReset = false; // Allow reset again if the switch is released
        }

        // Vertical motors reset
        if (digitalVert.getState() == PRESSED) {
            if (!verticalReset) {
                // Reset encoders and set target position to 0
                verticalLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                verticalRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                verticalLeft1.setTargetPosition(0);
                verticalRight1.setTargetPosition(0);
                verticalLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                verticalRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                verticalLeft1.setPower(0.5); // Set power to move the motor
                verticalRight1.setPower(0.5);
                verticalReset = true; // Mark reset as complete
            }
            // Stop motors if they're already at the target position
            if (verticalLeft1.isBusy() || verticalRight1.isBusy()) {
                // Motors are still moving, no action needed
            } else {
                verticalLeft1.setPower(0); // Stop motors once target is reached
                verticalRight1.setPower(0);
            }
        } else {
            verticalReset = false; // Allow reset again if the switch is released
        }
    }
}