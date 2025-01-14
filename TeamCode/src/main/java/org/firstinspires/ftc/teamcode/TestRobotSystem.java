package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "TestRobotSystem")
@Disabled
public class TestRobotSystem extends LinearOpMode {
    TankDriveTrainUtility dt = new TankDriveTrainUtility();
    TransferFinal tf = new TransferFinal();

    @Override
    public void runOpMode() {
        dt.initialize(hardwareMap);
        tf.servoInitializationAuto(hardwareMap, 0.7);
        waitForStart();
        while (opModeIsActive()) {

            //Drive

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            dt.moveRobot(drive, turn);

            telemetry.addData("drive %f", drive);
            telemetry.addData("turn %f", turn);
            telemetry.update();

            // Gamepad Button Simplified:
            boolean leftBumper = gamepad1.left_bumper;
            boolean rightBumper = gamepad1.right_bumper;
            boolean dpadUp = gamepad1.dpad_up;

            telemetry.addData("vertical left %f", tf.verticalLeft.getCurrentPosition());
            telemetry.addData("vertical right %f", tf.verticalRight.getCurrentPosition());
            telemetry.update();

            if (gamepad1.dpad_up) {
                tf.pickupSample();
                tf.transferSampleToBasket();
            }
            if (gamepad1.dpad_down) {
                tf.specimenPickup();
            }
            if (gamepad1.dpad_right) {
                tf.hangSpecimen();
            }
            if (gamepad1.dpad_left) {
                tf.finishHang();
            }
            if (gamepad1.right_bumper) {
                tf.dropSample();
            }
            if (gamepad1.a) {
                dt.arcRobot(-55.0, 20.0, 0.8);
                dt.arcRobot(55.0, 27.0, 0.8);
                sleep(500);
                dt.arcRobot(-55.0, -27.0, 0.8);
                dt.arcRobot(55.0, -20.0, 0.8);
            }
            if (gamepad1.b) {
                //tf.specimenPickup();
                ///Orient Specimen
                //tf.hangSpecimen();
                ///Drive forward 8 inches
                dt.arcRobot(-55.0, 22.0, 1.0);
                dt.arcRobot(55.0, 35.0, 1.0);

            }
        }
    }
}

