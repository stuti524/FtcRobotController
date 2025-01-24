/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 *  This OpMode illustrates the concept of driving an autonomous path based on Gyro (IMU) heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code uses the Universal IMU interface so it will work with either the BNO055, or BHI260 IMU.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *  The REV Logo should be facing UP, and the USB port should be facing forward.
 *  If this is not the configuration of your REV Control Hub, then the code should be modified to reflect the correct orientation.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: This code implements the requirement of calling setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver Station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  https://ftc-docs.firstinspires.org/field-coordinate-system
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="AutoGyroDefault", group="Robot")

public class AutoGyroREdgeCenterLine extends LinearOpMode {

    /* Declare OpMode members. */
    private IMU             imu         = null;   // Control/Expansion Hub IMU
    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4;//3.7346457 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.85;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.5;     // Max turn speed to limit turn rate.
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.03;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable.

    boolean specimenAuto                           = true;

    TankDriveTrainUtility dt = new TankDriveTrainUtility();
    TransferFinal tf = new TransferFinal();


    @Override
    public void runOpMode() {

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        //Initialize Drivetrain Motors and Servos
        dt.initialize(hardwareMap);
        tf.servoInitializationAuto(hardwareMap, 0.8, specimenAuto);

        // Wait for the game to start (Display Gyro value and Basket and Specimen Auto while waiting)
        while (opModeInInit()) {
            if (gamepad1.right_bumper)
                specimenAuto = true;
            if (gamepad1.left_bumper)
                specimenAuto = false;
            if (gamepad1.left_bumper || gamepad1.right_bumper) {
                tf.servoInitializationAuto(hardwareMap, 0.8, specimenAuto);
            }

            if (specimenAuto)
                telemetry.addData("AutoWithSpecimen", ")");
            else
                telemetry.addData("AutoWithBasket", "(");
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }
        //Reset the heading
        imu.resetYaw();

        /// Hang the first specimen (common between basket and specimen auto)


        /// Run one of the following 2 Auto paths: Only Specimen scoring OR Sample scoring in basket
        if (specimenAuto)
            runAutoWithSpecimen(); //Get specimen and hang them
        else
            runAutoWithBasket(); //TODO: Auto to get "Yellow" samples to score in high basket

        /// End Game Path with telemetry
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(100);  // Pause to display last telemetry message.
    }

    public void runAutoWithSpecimen () {
        ///Hang First Specimen
        driveStraight(DRIVE_SPEED, 30.5, 0.0, false, true, false);  // Drive forward 31 inches
        driveStraight(DRIVE_SPEED-0.2, 2.5, 0.0, false, false, false);  // Drive to get flush with submersible bar
        tf.finishHang();
        ///Drive backward 20 inches
        driveStraight(DRIVE_SPEED, -20, 0.0, false, false, false);
        ///Turn at a 50 degree angle to the right
        turnToHeading(TURN_SPEED, -50, HEADING_THRESHOLD);
        ///Drive forward 63 inches at a 20 degree angle to the right
        driveStraight(1, 30, -50, false, false, false);
        driveStraight(1, 28, -20,false, false, false);
        ///Reorient the robot to 0 degrees
        turnToHeading(TURN_SPEED, 0, 5);
        ///Go backwards 61 inches
        driveStraight(DRIVE_SPEED , -51, 0,true, false, false); //-51.5 old distance
        driveStraight(DRIVE_SPEED-0.2,  -5 , 0, true, false, false); //old speed: drive_speed-0.1
        ///Pickup the preset specimen from the wall
        tf.specimenPickup();
        ///Orient Specimen #2
        tf.hangSpecimen();
        ///Hang second specimen
        dt.arcRobot(-55.0, 22.0, 1.0);
        dt.arcRobot(55.0, 32.0, 1.0); //35 before
        tf.finishHang();
        ///Pickup third specimen from the wall
        dt.arcRobot(-55.0, -30.0, 1.0);
        dt.arcRobot(55.0, -14, 1.0); //-17 before
//        tf.specimenPickup();
//        ///Orient Specimen #3
//        tf.hangSpecimen();
//        ///Hang third specimen
//        dt.arcRobot(-55.0, 18.0, 1.0);
//        dt.arcRobot(55.0, 34.0, 1.0); //-21
//        tf.finishHang();
    }

    public void runAutoWithBasket() {
        ///Go back 15 inches
        driveStraight(DRIVE_SPEED , 11, 0, true, false, true);
        tf.verticalLeft.setTargetPosition(2100);
        tf.verticalRight.setTargetPosition(2100);
        while(tf.verticalLeft.isBusy() || tf.verticalRight.isBusy()) {
        }
        tf.controlTransfer(TransferFinal.TransferStates.TRANSFER_MIDDLE, TransferFinal.TransferStates.TRANSFER_CLOSE, TransferFinal.TransferStates.GIMBLE_HANG);
        tf.dropSample();
        tf.controlTransfer(TransferFinal.TransferStates.TRANSFER_HANG, TransferFinal.TransferStates.TRANSFER_OPEN, TransferFinal.TransferStates.GIMBLE_HANG);


        // TODO FOR BASKET AUTO: When you want to arc, use this function instead of arcRobot()
        arcWithBasket(-55.0, 22.0, 1.0, true);

        ///Go forward y inches
        ///Reorient to 0
        ///Go forward 7 inches
        ///Pickup the first preset sample from the ground
        ///Transfer sample to the transfer claw
        ///Go back -30 inches in at a 45 angle
        ///Drop sample into the high basket
        ////Scoring the 2nd Sample
        ///Reorient to 0
        ///Go forward q inches and lower the vertical slides to 0
        ///Pickup the first preset sample from the ground
        ///Transfer sample to the transfer claw
        ///Go back r in at a s angle
        ///Drop the sample into the high basket
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
     *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed, double distance, double heading, boolean brake, boolean readyToHang, boolean extendHori) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftTarget = dt.leftBackDrive.getCurrentPosition() + moveCounts;
            rightTarget = dt.rightBackDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            dt.leftBackDrive.setTargetPosition(leftTarget);
            dt.rightBackDrive.setTargetPosition(rightTarget);
            dt.leftFrontDrive.setTargetPosition(leftTarget);
            dt.rightFrontDrive.setTargetPosition(rightTarget);

            dt.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            dt.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            dt.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            dt.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (brake) {
                dt.leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                dt.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                dt.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                dt.rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else {
                dt.leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                dt.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                dt.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                dt.rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            if (extendHori){
                tf.horizontalMotor.setTargetPosition(-2800);
                tf.controlIntake(TransferFinal.IntakeStates.INTAKE_MIDDLE, TransferFinal.IntakeStates.GRAB_OPEN, TransferFinal.IntakeStates.GIMBLE_NINETY);
            }

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            dt.moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (
                    opModeIsActive() &&
                    ((dt.leftBackDrive.isBusy() && dt.rightBackDrive.isBusy()) //||
                   // (leftBackDrive.getVelocity() > 20 || leftBackDrive.getVelocity() < -20) ||
                   // (rightBackDrive.getVelocity() > 20 || rightBackDrive.getVelocity() < -20))

            )){

                // Determine required steering to keep on heading
                dt.turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    dt.turnSpeed *= -1.0;
                // if you want to extend vertical up in parallel with driving
                if (readyToHang) {
                    tf.hangSpecimen();
                    readyToHang = false;
                }

                // Apply the turning correction to the current driving speed.
                dt.moveRobot(dt.driveSpeed, dt.turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            dt.moveRobot(0, 0);
            dt.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            dt.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            dt.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            dt.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading, double headingTolerance) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
          /*
          while (
           opModeisActive AND
           error outside threshold AND
           velocity is too fast
            )
            {}
          */

        while (
                opModeIsActive() &&
                        ((Math.abs(headingError) > headingTolerance) ||
                                (dt.leftFrontDrive.getVelocity() > 10 || dt.leftFrontDrive.getVelocity() < -10)
                        )
        ) {

            // Determine required steering to keep on heading
            dt.turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            dt.turnSpeed = Range.clip(dt.turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            dt.moveRobot(0, dt.turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        dt.moveRobot(0, 0);
    }

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
     *  This function is useful for giving the robot a moment to stabilize its heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            dt.turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            dt.turnSpeed = Range.clip(dt.turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            dt.moveRobot(0, dt.turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        dt.moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      dt.leftBackDrive.getCurrentPosition(),
                    dt.rightBackDrive.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, dt.turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", dt.leftSpeed, dt.rightSpeed);
        telemetry.addData("Vertical Left Motor Position", tf.verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical Right Motor Position", tf.verticalRight.getCurrentPosition());
        double vlCurrent = ((DcMotorEx)tf.verticalLeft).getCurrent(CurrentUnit.MILLIAMPS);
        double vrCurrent = ((DcMotorEx)tf.verticalRight).getCurrent(CurrentUnit.MILLIAMPS);
        double hmCurrent = ((DcMotorEx)tf.horizontalMotor).getCurrent(CurrentUnit.MILLIAMPS);
        telemetry.addData("Currents for VL: ","%f, VR: %f, H: %f", vlCurrent, vrCurrent, hmCurrent);
        telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void arcWithBasket(double angle, double length, double speed, boolean readyToBasket) {
        dt.arcStart(angle, length, speed);

        // do something else while arc is busy
        if (readyToBasket){
            tf.verticalLeft.setTargetPosition(2100);
            tf.verticalRight.setTargetPosition(-2100);
            tf.controlTransfer(TransferFinal.TransferStates.TRANSFER_MIDDLE, TransferFinal.TransferStates.TRANSFER_CLOSE, TransferFinal.TransferStates.GIMBLE_BASKET);
            readyToBasket = false;
        }
        while (dt.rightFrontDrive.isBusy() || dt.leftFrontDrive.isBusy()) {
        }

        dt.leftFrontDrive.setPower(0);
        dt.leftBackDrive.setPower(0);
        dt.rightFrontDrive.setPower(0);
        dt.rightBackDrive.setPower(0);

    }
}
