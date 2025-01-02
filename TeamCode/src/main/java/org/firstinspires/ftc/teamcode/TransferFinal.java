package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TransferFinal {
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

    //Vertical Slide Motors
    public DcMotor verticalLeft = null;
    public DcMotor verticalRight = null;

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

    public void servoInitializationAuto(HardwareMap hMap, double verticalPower) {
        //Defining Servos with Hardware Maps:
        //Ascent Servos
        this.servostop2 = hMap.get(Servo.class, "cs1");
        this.servostop1 = hMap.get(Servo.class, "cs3");

        //Intake Servos
        this.rotateServo = hMap.get(Servo.class, "es3");
        this.grabServo = hMap.get(Servo.class, "es5");
        this.gimbleServo = hMap.get(Servo.class, "es1");

        //Transfer Servos
        this.TrotateServo = hMap.get(Servo.class, "cs5");
        this.TgrabServo = hMap.get(Servo.class, "cs2");
        this.TgimbleServo = hMap.get(Servo.class, "cs4");

        //Vertical Motors
        this.verticalLeft = hMap.get(DcMotor.class, "em1");
        this.verticalRight = hMap.get(DcMotor.class, "em2");

        //Initialize Servos, TODO: Include initialize values in enum definition
        this.rotateServo.setPosition(0.95);
        this.gimbleServo.setPosition(1.0);
        this.grabServo.setPosition(0.0);
        this.TrotateServo.setPosition(0.625);
        this.TgimbleServo.setPosition(0.75);
        this.TgrabServo.setPosition(0.0);

        this.servostop2.setPosition(0.20);
        this.servostop1.setPosition(1.0);

        //Initialize Vertical Motors
        this.verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.verticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.verticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.verticalLeft.setTargetPosition(0);
        this.verticalRight.setTargetPosition(0);

        this.verticalLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.verticalRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.verticalLeft.setPower(verticalPower);
        this.verticalRight.setPower(verticalPower);
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
        this.controlTransfer(TransferRotateStates.TRANSFER_UP, TransferGrabStates.TRANSFER_OPEN, TransferGimbleStates.TRANSFER_CENTER);
        // 2. Grab the specimen
        this.controlTransfer(TransferRotateStates.TRANSFER_UP, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.TRANSFER_CENTER);
        this.verticalLeft.setTargetPosition(300);
        this.verticalRight.setTargetPosition(-300);
        sleep(50);
        // 3. Rotate the transfer back
        // controlTransfer(TransferRotateStates.TRANSFER_DOWN, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.TRANSFER_CENTER);
    }

    public void hangSpecimen() {
        //1. Intialize transfer in the up position
        //this.controlTransfer(TransferRotateStates.TRANSFER_UP, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.TRANSFER_CENTER);
        //2. Raise vertical while transfer rotates into down position
        //this.controlTransfer(TransferRotateStates.TRANSFER_HANG, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.GIMBLE_HANG);
        this.controlTransfer(TransferRotateStates.TRANSFER_HANG, TransferGrabStates.TRANSFER_ADJUST, TransferGimbleStates.GIMBLE_HANG);
        this.controlTransfer(TransferRotateStates.TRANSFER_HANG, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.GIMBLE_HANG);
    }

    public void finishHang() {
        this.verticalLeft.setTargetPosition(900);
        this.verticalRight.setTargetPosition(-900);
        this.controlTransfer(TransferRotateStates.TRANSFER_HANG, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.GIMBLE_HANG);
        // //3. Let go of specimen
        this.controlTransfer(TransferRotateStates.TRANSFER_HANG, TransferGrabStates.TRANSFER_OPEN, TransferGimbleStates.GIMBLE_HANG);
        // //4. Retract vertical while fliiping transfer to up position
        this.verticalLeft.setTargetPosition(500);
        this.verticalRight.setTargetPosition(-500);
        this.controlTransfer(TransferRotateStates.TRANSFER_UP, TransferGrabStates.TRANSFER_OPEN, TransferGimbleStates.TRANSFER_CENTER);
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
            if (this.grabServo.getPosition() != 0) {
                sleep(100);
                this.grabServo.setPosition(0);
            }
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
            this.TgimbleServo.setPosition(0.75);
        }
        sleep(250);

    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException var4) {
            Thread.currentThread().interrupt();
        }

    }
}
