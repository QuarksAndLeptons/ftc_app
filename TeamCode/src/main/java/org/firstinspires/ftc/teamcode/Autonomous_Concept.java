package org.firstinspires.ftc.teamcode;
//Import standard FTC libraries

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

//Import hardware libraries
//Import Gyro
//Import special FTC-related libraries
//Import navigation libraries


@Autonomous(name = "Concept Autonomous", group = "Linear Opmode")
public class Autonomous_Concept extends LinearOpMode {

    //Initialize elapsed time object
    private ElapsedTime runtime = new ElapsedTime();

    //ROBOT HARDWARE
    //Instantiate chassis motors
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor leftMotor2;
    private DcMotor rightMotor2;
    private DcMotor dropMotor;

    //Initlize encoder variables
    private double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Encoder
    private double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    private double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    //Run this code when the "init" button is pressed
    @Override
    public void runOpMode() {
        //Give the OK message
        telemetry.addData("Status", "Initializing motors");
        telemetry.update();

        //Initialize robot hardware
        //Begin with the chassis
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        leftMotor2 = hardwareMap.dcMotor.get("left_drive2");
        rightMotor2 = hardwareMap.dcMotor.get("right_drive2");
        //Reset the encoders on the chassis to 0
        //NOTE: Do not add leftMotor and rightMotor2 because there aren't any encoders on 'em
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Set the motor modes to normal
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Reverse the right motors so all motors move forward when set to a positive speed.
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);
        //Now initialize the drop motor
        dropMotor = hardwareMap.dcMotor.get("glyphdrop_motor");
        dropMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        driveForward(15);
    }




    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------
    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));

    }

    private String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    /**
     * Move the forward for a certain amount of time
     *
     * @param time  the amount of time to move forward in <b>seconds</b>
     * @param power the power of each of the motors
     */
    private void moveForward(double time, double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        leftMotor2.setPower(power);
        rightMotor2.setPower(power);
        sleep((int) (time * 1000.0));
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor2.setPower(0);
        rightMotor2.setPower(0);
    }


    /**
     * Drive forward a certain distance based on encoder values
     * @param distance the distance to drive forward in <b>inches</b>.
     */
    public void driveForward(double distance) {
        int iterations = 0;
        long newLeftTarget = leftMotor.getCurrentPosition() + (long) (distance * COUNTS_PER_INCH);
        long newRightTarget = rightMotor2.getCurrentPosition() - (long) (distance * COUNTS_PER_INCH);
        boolean leftIsMoving = true, rightIsMoving = true;

        while ((leftIsMoving||rightIsMoving) && opModeIsActive()){
            telemetry.addData("Status", "Driving forward " + distance + " inches");
            telemetry.addData("Left Motor Position", leftMotor.getCurrentPosition());
            telemetry.addData("Left Motor Speed", leftMotor.getPowerFloat());
            telemetry.addData("Right Motor Position", rightMotor2.getCurrentPosition());
            telemetry.addData("Right Motor Speed", rightMotor2.getPowerFloat());
            telemetry.addData("Iterations", ++iterations);
            if(leftIsMoving){ //Are the left motors moving
                //Check if the left motors are done moving
                if(Math.abs(newLeftTarget-leftMotor.getCurrentPosition())<200){
                    leftMotor.setPower(0);
                    leftMotor2.setPower(0);
                    leftIsMoving = false;
                }
                //If the motors aren't done moving, keep moving them
                else if (leftMotor.getCurrentPosition() > newLeftTarget) {
                    leftMotor.setPower(-.5);
                    leftMotor2.setPower(-.5);
                }
                else {
                    leftMotor.setPower(.5);
                    leftMotor2.setPower(.5);
                }
            }
            if(rightIsMoving){ //Are the right motors moving
                //Check if the right motors are done moving
                if(Math.abs(newRightTarget-rightMotor.getCurrentPosition())<200){
                    rightMotor.setPower(0);
                    rightMotor2.setPower(0);
                    leftIsMoving = false;
                }
                //If the motors aren't done moving, keep moving them
                else if (rightMotor.getCurrentPosition() > newRightTarget) {
                    rightMotor.setPower(-.5);
                    rightMotor2.setPower(-.5);
                }
                else {
                    rightMotor.setPower(.5);
                    rightMotor2.setPower(.5);
                }
            }
        }
    }

    private void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Determine new target position, and pass to motor controller
        newLeftTarget = leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = rightMotor2.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        leftMotor.setTargetPosition(newLeftTarget);
        leftMotor2.setTargetPosition(newLeftTarget);
        rightMotor.setTargetPosition(newRightTarget);
        rightMotor2.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        leftMotor.setPower(Math.abs(speed));
        leftMotor2.setPower(Math.abs(speed));
        rightMotor.setPower(Math.abs(speed));
        rightMotor2.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, AND at least one motor is running.
        while (opModeIsActive() &&
                runtime.seconds() < timeoutS &&
                leftMotor.isBusy() || rightMotor2.isBusy()) {
            // Display it for the driver.
            telemetry.addData("Left Target", "Running to", newLeftTarget);
            telemetry.addData("Right Target", "Running to", newRightTarget);
            telemetry.addData("leftMotor", "Running at", leftMotor.getCurrentPosition());
            telemetry.addData("rightMotor2", "Running at", rightMotor2.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        leftMotor.setPower(0);
        leftMotor2.setPower(0);
        rightMotor.setPower(0);
        rightMotor2.setPower(0);

        // Turn off RUN_TO_POSITION
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(1000);   // optional pause after each move

        telemetry.addData("Status", "Done moving forward");
        telemetry.update();
    }
}







