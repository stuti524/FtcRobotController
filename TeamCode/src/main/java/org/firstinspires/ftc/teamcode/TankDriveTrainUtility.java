package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TankDriveTrainUtility {
    public DcMotorEx leftFrontDrive   = null;
    public DcMotorEx leftBackDrive   = null;
    public DcMotorEx  rightFrontDrive  = null;
    public DcMotorEx  rightBackDrive  = null;

    public double  driveSpeed    = 0;
    public double  turnSpeed     = 0;
    public double  leftSpeed     = 0;
    public double  rightSpeed    = 0;

    public void initialize (HardwareMap hMap) {
        // Define and Initialize Motors
        this.leftFrontDrive  = hMap.get(DcMotorEx.class, "cm2");
        this.leftBackDrive  = hMap.get(DcMotorEx.class, "cm3");
        this.rightFrontDrive = hMap.get(DcMotorEx.class, "cm0");
        this.rightBackDrive = hMap.get(DcMotorEx.class, "cm1");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        this.leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        this.leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        this.rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        this.rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        this.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set the encoders for closed loop speed control
        this.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        this.driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        this.turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        this.leftSpeed  = drive - turn;
        this.rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(this.leftSpeed), Math.abs(this.rightSpeed));
        if (max > 1.0)
        {
            this.leftSpeed /= max;
            this.rightSpeed /= max;
        }

        this.leftBackDrive.setPower(this.leftSpeed);
        this.rightBackDrive.setPower(this.rightSpeed);
        this.leftFrontDrive.setPower(this.leftSpeed);
        this.rightFrontDrive.setPower(this.rightSpeed);
    }

    /**
     * This is (not) our function for arcing, a special type of movement that allows for turning while moving.
     *     Use the angle and length to determine where the robot will end up.
     *
     * @param angle
     * @param length
     * @param speed
     */
    public void arcRobot(double angle, double length, double speed) {

        this.arcStart(angle, length, speed);

        while (this.rightFrontDrive.isBusy() || this.leftFrontDrive.isBusy()) {
        }

//        this.leftFrontDrive.setPower(0);
//        this.leftBackDrive.setPower(0);
//        this.rightFrontDrive.setPower(0);
//        this.rightBackDrive.setPower(0);
    }

    public void arcStart(double angle, double length, double speed) {
        //\frac{c*sin*(90-b)}{\sin2b}
        Integer cpr = 28;
        double arcBias = 0.0; //NOT RECOMMENDED BY ftcchad.com
        double width = 17.25; //inches; this is thw width of the ROBOT
        double bias = 1.0;
        double gearratio = 19.2;
        double diameter = 3.7715;
        double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch -> counts per rotation / circumference
        double radius = ((length + arcBias) * Math.sin(Math.toRadians(90-angle)))/(Math.sin(Math.toRadians(2 * angle)));

        double conversion = cpi * bias;
        boolean exit = false;
        //telemetry.addData("radius", radius);
        //telemetry.update();
        //2\pi\left(r+a\right)\left(\frac{b}{180}\right)
        //2\pi\left(r-a\right)\left(\frac{b}{180}\right)
        //
        double rightMotor;
        double leftMotor;
        rightMotor = 2 * Math.PI * (radius - (width / 2)) * (angle / 180);
        leftMotor = 2 * Math.PI * (radius + (width / 2)) * (angle / 180);
        int rightd = (int) (Math.round(rightMotor * conversion));
        int leftd = (int) (Math.round(leftMotor * conversion));

        this.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //
        this.leftBackDrive.setTargetPosition(0);
        this.leftFrontDrive.setTargetPosition(0);
        this.rightBackDrive.setTargetPosition(0);
        this.rightFrontDrive.setTargetPosition(0);
        this.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.rightFrontDrive.setTargetPosition(this.rightFrontDrive.getCurrentPosition() + rightd);
        this.rightBackDrive.setTargetPosition(this.rightBackDrive.getCurrentPosition() + rightd);
        this.leftFrontDrive.setTargetPosition(this.leftFrontDrive.getCurrentPosition() + leftd);
        this.leftBackDrive.setTargetPosition(this.leftBackDrive.getCurrentPosition() + leftd);

        this.leftBackDrive.setPower(speed);
        this.leftFrontDrive.setPower(speed);
        this.rightBackDrive.setPower((rightMotor / leftMotor) * speed);
        this.rightFrontDrive.setPower((rightMotor / leftMotor) * speed);
    }

}
