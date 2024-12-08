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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    public DcMotorEx leftFrontDrive   = null;
    public DcMotorEx leftBackDrive   = null;
    public DcMotorEx  rightFrontDrive  = null;
    public DcMotorEx  rightBackDrive  = null;
    public DcMotor verticalLeft = null;
    public DcMotor verticalRight = null;
    public Servo servostop1 = null;
    public Servo servostop2 = null;

    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
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
    static final double     DRIVE_SPEED             = 0.5;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.3;     // Max turn speed to limit turn rate.
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.03;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable.

    enum VertE {
        ABOVE_CHAM,
        ON_CHAM,
        RESET_CHAM
    }
    //VertE state = VertE.ABOVE_CHAM;
    enum ClawServoE {
        //INTAKE_NO_DOWN_UP,
        CLAW_CLOSE,
        CLAW_OPEN
    }

    Servo rotateservo;
    Servo clawservo;
    ClawServoE clawservoe1= ClawServoE.CLAW_CLOSE;
    VertE vertchame= VertE.RESET_CHAM;
    //public boolean extended_vert = false;


    @Override
    public void runOpMode() {

        // Define and Initialize Motors
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "cm2");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "cm3");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "cm0");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "cm1");

        verticalLeft = hardwareMap.get(DcMotor.class, "em1");
        verticalRight = hardwareMap.get(DcMotor.class, "em2");

        clawservo = hardwareMap.get(Servo.class, "cs5");

        rotateservo = hardwareMap.get(Servo.class, "es3");

        servostop1 = hardwareMap.get(Servo.class, "cs3");
        servostop2 = hardwareMap.get(Servo.class, "cs1");

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Wall

        verticalLeft.setTargetPosition(0);
        verticalRight.setTargetPosition(0);

        verticalLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        verticalLeft.setPower(0.5);
        verticalRight.setPower(0.5);
        //rotateServo = hardwareMap.get(Servo.class, "es3");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        servostop1.setPosition(1.0);
        servostop2.setPosition(0.20);

        rotateservo.setPosition(0.87);

        // Set the encoders for closed loop speed control, and reset the heading.
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();


        driveStraight(DRIVE_SPEED, -24.0, 0.0, true);                       // Drive backward 24 inches
        hangSpecimen(1, -6.0);                                               // Hang Specimen 1
        driveStraight(DRIVE_SPEED, 19, 0.0, false);                         // Drive forward 19 inches
        turnToHeading (TURN_SPEED, -90.0);                                   // Turn 90 degrees to the left
        //holdHeading(TURN_SPEED, -90.0, 0.1);                        // Hold the turn for 0.1 seconds
        driveStraight(DRIVE_SPEED+0.2, -45.5, getHeading(), false);   // Go backwards 45.5 inches
        turnToHeading(TURN_SPEED, getHeading()-90.0);                        // Turn 90 degrees to the left
        //holdHeading(TURN_SPEED, getHeading(), 0.1);                         // Hold the turn for 0.1 seconds
        driveStraight(0.2, -11.6, getHeading(), false);                 // Drive backward 12 inches
        grabSpecimen();
        driveStraight(DRIVE_SPEED, 11.6, getHeading(), false);                        // Drive forward 12 inches
        turnToHeading(TURN_SPEED, getHeading()-90.0);                        // Turn 90 degrees to the left
        //holdHeading(TURN_SPEED, getHeading(), 0.1);                         // Hold the turn for 0.1 seconds
        driveStraight(DRIVE_SPEED, -53.5, getHeading(), false);                     // Drive backward 45 inches
        turnToHeading(TURN_SPEED, getHeading()-90.0);                        // Turn 90 degrees to the left
        //holdHeading(TURN_SPEED, getHeading(), 0.1);                         // Hold the turn for 0.1 seconds
        driveStraight(DRIVE_SPEED, -11.8, getHeading(), true);                      // Drive backward 45 inches
        hangSpecimen(2, -6.5);                                             // Hang Specimen 2
        // Park before Auto period ends
        driveStraight(0.8, 6, getHeading(), false);
        turnToHeading(0.8, 65.0);
        driveStraight(0.8, 44, 65, false);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        //sleep(1000);  // Pause to display last telemetry message.
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
    public void driveStraight(double maxDriveSpeed, double distance, double heading, boolean readyToHang) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftTarget = leftBackDrive.getCurrentPosition() + moveCounts;
            rightTarget = rightBackDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftBackDrive.setTargetPosition(leftTarget);
            rightBackDrive.setTargetPosition(rightTarget);
            leftFrontDrive.setTargetPosition(leftTarget);
            rightFrontDrive.setTargetPosition(rightTarget);

            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftBackDrive.isBusy() && rightBackDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;
                // if you want to extend vertical up in parallel with driving
                if (readyToHang) {
                    verticalLeft.setTargetPosition(1300);
                    verticalRight.setTargetPosition(-1300);
                    readyToHang = false;
                }

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    public void turnToHeading(double maxTurnSpeed, double heading) {

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
                        ((Math.abs(headingError) > HEADING_THRESHOLD) ||
                                (leftFrontDrive.getVelocity() > 10 || leftFrontDrive.getVelocity() < -10)
                        )
        ) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
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
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
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
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftBackDrive.setPower(leftSpeed);
        rightBackDrive.setPower(rightSpeed);
        leftFrontDrive.setPower(leftSpeed);
        rightFrontDrive.setPower(rightSpeed);
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
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftBackDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.addData("Vertical Left Motor Position", verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical Right Motor Position", verticalRight.getCurrentPosition());
        telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void hangSpecimen(int specimenNumber, double closeDistance){
        chamber(false, 1);
        telemetry.addData("Status: Hanging Specimen %d", specimenNumber);
        telemetry.update();
        driveStraight(DRIVE_SPEED, closeDistance, 0.0, false);
        chamber(false, 2);
        chamber(true, 0);
    }

    public void chamber(boolean open, double extended) {
        if (open){
            clawservoe1 = ClawServoE.CLAW_OPEN;
        }
        else{
            clawservoe1 = ClawServoE.CLAW_CLOSE;
        }

        if (clawservoe1 == ClawServoE.CLAW_CLOSE)
            clawservo.setPosition(1.2); // vertical extension claw is closed
        if (clawservoe1 == ClawServoE.CLAW_OPEN) {
            clawservo.setPosition(0); // vertical extension claw is open
            sleep(300);
        }

        if (extended == 0){
            vertchame = VertE.RESET_CHAM;
        }
        if (extended == 1){
            vertchame = VertE.ABOVE_CHAM;
        }
        if (extended == 2){
            vertchame = VertE.ON_CHAM;
        }


        // hardware calls
        if (vertchame == VertE.ABOVE_CHAM) {
            verticalLeft.setTargetPosition(1300);
            verticalRight.setTargetPosition(-1300);
        } else if (vertchame == VertE.ON_CHAM) {
            verticalLeft.setTargetPosition(900);
            verticalRight.setTargetPosition(-900);
        } else if (vertchame == VertE.RESET_CHAM) {
            verticalLeft.setTargetPosition(0);
            verticalRight.setTargetPosition(0);
        }
        while(verticalLeft.isBusy() || verticalRight.isBusy()){
            sendTelemetry(true);
        }
    }


    public void grabSpecimen() {
        sleep(100);
        wall(true, false); //claw = open. vertical slides = down
        sleep(300);
        wall(false, false); //claw = close. vertical slides = down
        sleep(300);
        wall(false, true);  //claw = close. vertical slides = up
        sleep(200);

    }


    public void wall(boolean open, boolean extended_vert) {
        if (open){
            clawservoe1 = ClawServoE.CLAW_OPEN;
        }
        else{
            clawservoe1 = ClawServoE.CLAW_CLOSE;
        }

        if (clawservoe1 == ClawServoE.CLAW_CLOSE) {
            clawservo.setPosition(1.2); //claw is closed
        }
        if (clawservoe1 == ClawServoE.CLAW_OPEN) {
            clawservo.setPosition(0); //claw is open
        }

        if (extended_vert) {
            verticalLeft.setTargetPosition(300);
            verticalRight.setTargetPosition(-300);
        } else {
            verticalLeft.setTargetPosition(0);
            verticalRight.setTargetPosition(0);
        }

        telemetry.addData("ClawServoPOS!", clawservo.getPosition());
        telemetry.update();

        telemetry.addData("Vertical Left Motor Position", verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical Right Motor Position", verticalRight.getCurrentPosition());

        telemetry.update();

    }
}
