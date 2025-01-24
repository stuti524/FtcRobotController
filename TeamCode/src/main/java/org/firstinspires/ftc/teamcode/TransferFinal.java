package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    public DcMotor horizontalMotor = null;
    //Enums:
    enum IntakeStates {
        INTAKE_DOWN(0.12),
        INTAKE_MIDDLE(0.35),
        INTAKE_UP(0.95),
        GRAB_CLOSE(0.56),
        GRAB_OPEN(0),
        GRAB_ADJUST(0.5),
        GIMBLE_CENTER(0.55),
        GIMBLE_NINETY(1);
        private final double position;  // Field to store the position

        IntakeStates(double value) {
            this.position = value;  // 'this.position' refers to the position field of the enum constant
        }

        public double value() {
            return position;
        }
    }

    enum TransferStates {
        TRANSFER_DOWN(0.7),   //1914
        TRANSFER_MIDDLE(0.250), //1080
        TRANSFER_UP(0.1),
        TRANSFER_HANG(0.5),  //1455  0.468
        TRANSFER_CLOSE(0.32),
        TRANSFER_OPEN(0.52),
        TRANSFER_ADJUST(0.35),
        TRANSFER_CENTER(0.30), //was 0.5, why?
        TRANSFER_NINETY(0.9),
        GIMBLE_HANG(0.5), // was 0.62, why?
        GIMBLE_BASKET(0.54);
        private final double position;  // Field to store the position

        // Constructor to initialize the position field with the passed value
        TransferStates(double value) {
            this.position = value;  // 'this.position' refers to the position field of the enum constant
//            double newMapping = Range.scale(1080, 800,2200,0,1);
        }

        // Getter method to retrieve the position
        public double value() {
            return position;
        }
    }


    public void servoInitializationAuto(HardwareMap hMap, double verticalPower, boolean specimenAuto) {
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

        this.horizontalMotor = hMap.get(DcMotor.class, "em0");

        //Initialize Servos:
        this.rotateServo.setPosition(IntakeStates.INTAKE_UP.value());
        this.gimbleServo.setPosition(IntakeStates.GIMBLE_NINETY.value());
        this.grabServo.setPosition(IntakeStates.GRAB_OPEN.value());
        if (specimenAuto){
            this.TrotateServo.setPosition(0.68);
        }
        else {
            this.TrotateServo.setPosition(0.5);
        }
        this.TgimbleServo.setPosition(TransferStates.GIMBLE_HANG.value());
        this.TgrabServo.setPosition(TransferStates.TRANSFER_CLOSE.value());

        this.servostop2.setPosition(0.12);
        this.servostop1.setPosition(1.0);

        //Initialize Vertical Motors
        this.verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.horizontalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.verticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.verticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.horizontalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.verticalLeft.setTargetPosition(0);
        this.verticalRight.setTargetPosition(0);
        this.horizontalMotor.setTargetPosition(0);

        this.verticalLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.verticalRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.horizontalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.verticalLeft.setPower(verticalPower);
        this.verticalRight.setPower(verticalPower);
        this.horizontalMotor.setPower(0.8);

    }
    public void servoInitializationTeleop(HardwareMap hMap, double verticalPower) {
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
//        this.verticalLeft = hMap.get(DcMotor.class, "em1");
//        this.verticalRight = hMap.get(DcMotor.class, "em2");

//        this.horizontalMotor = hMap.get(DcMotor.class, "em0");

//        Initialize Servos:
//        this.rotateServo.setPosition(IntakeStates.INTAKE_UP.value());
//        this.gimbleServo.setPosition(IntakeStates.GIMBLE_NINETY.value());
//        this.grabServo.setPosition(IntakeStates.GRAB_OPEN.value());
//        this.TrotateServo.setPosition(TransferStates.TRANSFER_HANG.value());
//        this.TgimbleServo.setPosition(TransferStates.GIMBLE_HANG.value());
//        this.TgrabServo.setPosition(TransferStates.TRANSFER_CLOSE.value());
//
////        this.servostop2.setPosition(0.20);
////        this.servostop1.setPosition(1.0);
//
//        //Initialize Vertical Motors
//        this.verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        this.verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        this.horizontalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        this.verticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.verticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        this.horizontalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        this.verticalLeft.setTargetPosition(0);
//        this.verticalRight.setTargetPosition(0);
////        this.horizontalMotor.setTargetPosition(0);
//
//        this.verticalLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        this.verticalRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        this.horizontalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        this.verticalLeft.setPower(verticalPower);
//        this.verticalRight.setPower(verticalPower);
//        this.horizontalMotor.setPower(1);

    }
    public void showTelemetry() {
//        telemetry.addData("RotateServoPOS", this.rotateServo.getPosition());
//        telemetry.addData("GimbleServoPOS", this.gimbleServo.getPosition());
//        telemetry.addData("GrabServoPOS", this.grabServo.getPosition());
//        telemetry.addData("TransferRotateServoPOS", this.TrotateServo.getPosition());
//        telemetry.addData("TransferGimbleServoPOS", this.TgimbleServo.getPosition());
//        telemetry.addData("TransferGrabServoPOS", this.TgrabServo.getPosition());
//        double vlCurrent = ((DcMotorEx)this.verticalLeft).getCurrent(CurrentUnit.MILLIAMPS);
//        double vrCurrent = ((DcMotorEx)this.verticalRight).getCurrent(CurrentUnit.MILLIAMPS);
//        double hmCurrent = ((DcMotorEx)this.horizontalMotor).getCurrent(CurrentUnit.MILLIAMPS);
//        telemetry.addLine(String.format("Currents for VL: %s, VR: %s, H: %s", vlCurrent, vrCurrent, hmCurrent));
//        telemetry.update();
    }
    public void specimenPickup() {
        this.verticalLeft.setPower(1);
        this.verticalRight.setPower(1);
        // 1. Initialization
        //controlTransfer(TransferRotateStates.TRANSFER_DOWN, TransferGrabStates.TRANSFER_OPEN, TransferGimbleStates.TRANSFER_NINETY);
        // 2. Orient the transfer into the correct position
        this.controlTransfer(TransferStates.TRANSFER_UP, TransferStates.TRANSFER_OPEN, TransferStates.TRANSFER_CENTER);
        // 2. Grab the specimen
        this.controlTransfer(TransferStates.TRANSFER_UP, TransferStates.TRANSFER_CLOSE, TransferStates.TRANSFER_CENTER);
        this.verticalLeft.setTargetPosition(585);
        this.verticalRight.setTargetPosition(-585);
        sleep(50);
        // 3. Rotate the transfer back
        // controlTransfer(TransferRotateStates.TRANSFER_DOWN, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.TRANSFER_CENTER);

    }

    public void hangSpecimen() {
        this.verticalLeft.setPower(1);
        this.verticalRight.setPower(1);
        //1. Intialize transfer in the up position
        //this.controlTransfer(TransferRotateStates.TRANSFER_UP, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.TRANSFER_CENTER);
        //2. Raise vertical while transfer rotates into down position
        //this.controlTransfer(TransferRotateStates.TRANSFER_HANG, TransferGrabStates.TRANSFER_CLOSE, TransferGimbleStates.GIMBLE_HANG);
        this.verticalLeft.setTargetPosition(975);
        this.verticalRight.setTargetPosition(-975);
        this.controlTransfer(TransferStates.TRANSFER_HANG, TransferStates.TRANSFER_ADJUST, TransferStates.GIMBLE_HANG);
        this.controlTransfer(TransferStates.TRANSFER_HANG, TransferStates.TRANSFER_CLOSE, TransferStates.GIMBLE_HANG);
    }

    public void finishHang() {
        this.verticalLeft.setPower(1);
        this.verticalRight.setPower(1);
        this.verticalLeft.setTargetPosition(1900);
        this.verticalRight.setTargetPosition(-1900);
        while(this.verticalLeft.isBusy() || this.verticalRight.isBusy()) {
        }
//        this.controlTransfer(TransferStates.TRANSFER_HANG, TransferStates.TRANSFER_CLOSE, TransferStates.GIMBLE_HANG);
        // //3. Let go of specimen
        this.controlTransfer(TransferStates.TRANSFER_HANG, TransferStates.TRANSFER_OPEN, TransferStates.GIMBLE_HANG);
        // //4. Retract vertical while fliiping transfer to up position
        this.verticalLeft.setTargetPosition(975);
        this.verticalRight.setTargetPosition(-975);
        this.controlTransfer(TransferStates.TRANSFER_UP, TransferStates.TRANSFER_OPEN, TransferStates.TRANSFER_CENTER);
        this.verticalLeft.setTargetPosition(0);
        this.verticalRight.setTargetPosition(0);
        while (this.verticalLeft.isBusy() || this.verticalRight.isBusy()) {
        }
        this.verticalLeft.setPower(0);
        this.verticalRight.setPower(0);
        showTelemetry();
    }
    public void pickupSample() {
        //this.controlIntake(IntakeRotateStates.INTAKE_MIDDLE, IntakeGrabStates.GRAB_OPEN, IntakeGimbleStates.GIMBLE_NINETY);
        this.controlIntake(IntakeStates.INTAKE_DOWN, IntakeStates.GRAB_CLOSE, IntakeStates.GIMBLE_NINETY);

    }
    public void sampleTransfer() {
        // 1. Position the intake claw right above the sample
        // controlIntake(IntakeRotateStates.INTAKE_MIDDLE, IntakeGrabStates.GRAB_OPEN, IntakeGimbleStates.GIMBLE_NINETY);
        //sleep(250);
        // // 2. Pickup the sample with the intake claw
        // controlIntake(IntakeRotateStates.INTAKE_DOWN, IntakeGrabStates.GRAB_CLOSE, IntakeGimbleStates.GIMBLE_NINETY);
        //sleep(250);
        // 3. Position the rotate servo to the correct position

        this.controlIntake(IntakeStates.INTAKE_UP, IntakeStates.GRAB_CLOSE, IntakeStates.GIMBLE_NINETY);
        // sleep(250);
        // 4. Loose power to position the sample correctly in the claw
        this.controlIntake(IntakeStates.INTAKE_UP, IntakeStates.GRAB_ADJUST, IntakeStates.GIMBLE_NINETY);
        //sleep(250);
        // 5. Bring the transfer directly over the sample
        this.controlTransfer(TransferStates.TRANSFER_UP, TransferStates.TRANSFER_OPEN, TransferStates.TRANSFER_NINETY);
        //sleep(500);
        // 6. Grab the sample with the transfer claw
        this.controlTransfer(TransferStates.TRANSFER_DOWN, TransferStates.TRANSFER_OPEN, TransferStates.TRANSFER_NINETY);
        this.controlTransfer(TransferStates.TRANSFER_DOWN, TransferStates.TRANSFER_CLOSE, TransferStates.TRANSFER_NINETY);
        //sleep(250);
        // 7. Rotate the transfer while holding the sample
        this.controlTransfer(TransferStates.TRANSFER_UP, TransferStates.TRANSFER_CLOSE, TransferStates.TRANSFER_NINETY);
//        this.controlTransfer(TransferStates.TRANSFER_UP, TransferStates.TRANSFER_CLOSE, TransferStates.TRANSFER_CENTER);
    }

    public void transferSampleToBasket() {
        this.verticalLeft.setPower(1);
        this.verticalRight.setPower(1);
        // 1. Position the intake claw right above the sample
        // controlIntake(IntakeRotateStates.INTAKE_MIDDLE, IntakeGrabStates.GRAB_OPEN, IntakeGimbleStates.GIMBLE_NINETY);
        // //sleep(250);
        // // 2. Pickup the sample with the intake claw
        // controlIntake(IntakeRotateStates.INTAKE_DOWN, IntakeGrabStates.GRAB_CLOSE, IntakeGimbleStates.GIMBLE_NINETY);
        //sleep(250);
        // 3. Position the rotate servo to the correct position
        this.controlIntake(IntakeStates.INTAKE_UP, IntakeStates.GRAB_CLOSE, IntakeStates.GIMBLE_NINETY);
        // sleep(250);
        // 4. Loose power to position the sample correctly in the claw
        this.controlIntake(IntakeStates.INTAKE_UP, IntakeStates.GRAB_ADJUST, IntakeStates.GIMBLE_NINETY);
        //sleep(250);
        // 5. Bring the transfer directly over the sample
        this.controlTransfer(TransferStates.TRANSFER_UP, TransferStates.TRANSFER_OPEN, TransferStates.TRANSFER_NINETY);
        //this.controlTransfer(TransferStates.TRANSFER_DOWN, TransferStates.TRANSFER_OPEN, TransferStates.TRANSFER_NINETY);
        //sleep(1000);
        // 6. Grab the sample with the transfer claw
        this.controlTransfer(TransferStates.TRANSFER_DOWN, TransferStates.TRANSFER_OPEN, TransferStates.TRANSFER_NINETY);
        this.controlTransfer(TransferStates.TRANSFER_DOWN, TransferStates.TRANSFER_CLOSE, TransferStates.TRANSFER_NINETY);
        //sleep(250);
        this.verticalLeft.setTargetPosition(2100); //MAX VERTICAL POSITION
        this.verticalRight.setTargetPosition(-2100); //MAX VERTICAL POSITION
        // 7. Rotate the transfer while holding the sample
        this.controlTransfer(TransferStates.TRANSFER_MIDDLE, TransferStates.TRANSFER_CLOSE, TransferStates.GIMBLE_BASKET);
    }

    public void dropSample() {
        this.controlTransfer(TransferStates.TRANSFER_MIDDLE, TransferStates.TRANSFER_OPEN, TransferStates.GIMBLE_BASKET);
    }


    public void controlIntake(IntakeStates rotate, IntakeStates grab, IntakeStates gimble) {
        //Hardware Calls for Intake Servos:
        //Intake Rotate Servo:
        if (this.rotateServo.getPosition() != rotate.value()){
            this.rotateServo.setPosition(rotate.value());
            sleep(250);
        }
        //Intake Grab Servo:
        if (this.grabServo.getPosition() != grab.value()){
            this.grabServo.setPosition(grab.value());
            sleep(200);
        }

        if (grab == IntakeStates.GRAB_ADJUST) {
            this.grabServo.setPosition(IntakeStates.GRAB_ADJUST.value());
            sleep(200);
            this.grabServo.setPosition(IntakeStates.GRAB_CLOSE.value());
        }
        //Intake Gimble Servo:
        if (this.gimbleServo.getPosition() != gimble.value()){
            this.gimbleServo.setPosition(gimble.value());
            sleep(200);
        }
    }

    public void controlTransfer(TransferStates rotate, TransferStates grab, TransferStates gimble) {
        // Hardware Calls for Transfer Servos:
        // Transfer Rotate Servo:
        if (this.TrotateServo.getPosition() != rotate.value()){
            this.TrotateServo.setPosition(rotate.value());
            sleep(500);
        }

        // Transfer Grab Servo:
        if(this.TgrabServo.getPosition() != grab.value()){
            this.TgrabServo.setPosition(grab.value());
            sleep(200);
        }
        if (grab == TransferStates.TRANSFER_CLOSE) {
            this.TgrabServo.setPosition(0); //Close Claw
            if (this.grabServo.getPosition() != IntakeStates.GRAB_OPEN.value()) { //
                //sleep(100);
                this.grabServo.setPosition(IntakeStates.GRAB_OPEN.value());
                sleep(200);
            }
        }

        // Transfer Gimbal Servo:
        if(this.TgimbleServo.getPosition() != gimble.value()) {
            this.TgimbleServo.setPosition(gimble.value());
            sleep(250);
        }

    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException var4) {
            Thread.currentThread().interrupt();
        }

    }
}
