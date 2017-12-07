package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

/**
 * Created by Micah on 12/5/17.
 */

public abstract class Autonomous extends LinearOpMode {
    //Initialize and instantiate vuforia variables
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    //Initialize elapsed time object
    protected ElapsedTime runtime = new ElapsedTime();

    // The IMU sensor object
    protected BNO055IMU imu;

    // Orientation and acceleration variables from the built in 9-axis accelerometer
    protected Orientation angles;
    protected Acceleration gravity;

    //ROBOT HARDWARE
    //Instantiate chassis motors

    protected DcMotorEx leftMotor;
    protected DcMotorEx rightMotor;
    protected DcMotorEx leftMotor2;
    protected DcMotorEx rightMotor2;
    protected DcMotorEx dropMotor;
    //Instantiate servos
    protected Servo color_servo;
    protected Servo rotation_servo;
    //Instantiate sensors
    ColorSensor blueSensorColor;
    ColorSensor redSensorColor;

    //Initlize encoder variables
    protected double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Encoder
    protected double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    protected double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    protected double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    //Initialize Vuforia variables
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    /**
     * Initializes all of the motors and servos.
     * <b>Be sure to call this before waitForStart()</b>
     */
    protected void initializeHardware(){
        //Give the OK message
        telemetry.addData("Status", "Initializing motors");
        telemetry.update();

        //Initialize robot hardware
        //Begin with the chassis
        leftMotor = (DcMotorEx)hardwareMap.get(DcMotor.class,"leftFront");
        rightMotor = (DcMotorEx)hardwareMap.get(DcMotor.class,"rightFront");
        leftMotor2 = (DcMotorEx)hardwareMap.get(DcMotor.class,"leftRear");
        rightMotor2 = (DcMotorEx)hardwareMap.get(DcMotor.class,"rightRear");
        //Reset the encoders on the chassis to 0
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Set the motor modes to normal
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Reverse the right motors so all motors move forward when set to a positive speed.
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor2.setDirection(DcMotor.Direction.REVERSE);
        //Now initialize the drop motor
        dropMotor = (DcMotorEx)hardwareMap.get(DcMotor.class,"glyphDropMotor");
        dropMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Initialize the servos
        color_servo = hardwareMap.get(Servo.class, "jewelServo");
        rotation_servo = hardwareMap.get(Servo.class, "jewelRotationServo");
        //Initialize sensors
        blueSensorColor = hardwareMap.get(ColorSensor.class, "BlueColorSensor");
        redSensorColor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "RedColorSensor");


        //Initialize Vuforia extension
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersV = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parametersV.vuforiaLicenseKey = "AfRvW7T/////AAAAGSZSz12Y5EPmopiqX9Cc17EX1TB9P/jfFGOFM0F4JDpeOV7v14oYyRCIzW09fiRPCvR2ivZcx3tHGuJTENamTxdwxZYSE72C9xuk7pmCjFeP5wfD3zF7bgYCrcVKk6Piahys8ccRRg93Dw4tC0kqkwrW5iz+u1x6+o6CctbGc8nxY3AzaH3W9HTU+QeGLv1xAx05YFOHwSgzNn9mZJYu2rYu2pBdKmb2Y918AlOwEC8QvbSARqI4aCKQle+Nplm/dBTFWO1p6sBIA8A7HHeb2vKcwHfkD10HEzDsZYuQNVxERAJ+7hfmmRLHLmtQJqXTWsQ6mMlKruGtm+s8m+4LhIEZy3dHnX4131J4bvSz1V6f";
        parametersV.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; //Set camera
        vuforia = ClassFactory.createVuforiaLocalizer(parametersV);
        //Get the assets for Vuforia
        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        //Initialize Gryo
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parametersG = new BNO055IMU.Parameters();
        parametersG.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersG.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersG.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersG.loggingEnabled = true;
        parametersG.loggingTag = "IMU";
        parametersG.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersG);

        // Set up our telemetry dashboard
        composeTelemetry();
    }

    /**
     * Begin Vuforia tracking and caliberate gyro.
     * <b>Call this <i>after</i> waitForStart()</b>
     */
    protected void startAdvancedSensing(){
        //Do some calibration and activation
        // Calibrate Gyro
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        //Activate Vuforia
        relicTrackables.activate();
    }

    protected void composeTelemetry() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(() -> {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = imu.getGravity();
        });
        telemetry.addLine()
                .addData("status", () -> imu.getSystemStatus().toShortString())
                .addData("calib", () -> imu.getCalibrationStatus().toString());
        telemetry.addLine()
                .addData("heading", () -> formatAngle(angles.angleUnit, angles.firstAngle))
                .addData("roll", () -> formatAngle(angles.angleUnit, angles.secondAngle))
                .addData("pitch", () -> formatAngle(angles.angleUnit, angles.thirdAngle));
        telemetry.addLine()
                .addData("grvty", () -> gravity.toString())

                .addData("mag", () -> String.format(Locale.getDefault(), "%.3f",
                        Math.sqrt(gravity.xAccel * gravity.xAccel
                                + gravity.yAccel * gravity.yAccel
                                + gravity.zAccel * gravity.zAccel)));
    }

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));

    }

    private String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /**
     * Basic Drivetrain
     *
     * @param power the power of each of the motors
     */

    protected void rightDrive(double power) {
            rightMotor.setPower(power);
            rightMotor2.setPower(power);
        }

    protected void leftDrive(double power) {
        leftMotor.setPower(power);
        leftMotor2.setPower(power);

    }

    /**
     * Move the forward for a certain amount of time
     *
     * @param time  the amount of time to move forward in <b>seconds</b>
     * @param power the power of each of the motors
     */
    protected void driveForwardTime(double time, double power) {
        leftDrive(power);
        rightDrive(power);
        sleep((int) (time * 1000.0));
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor2.setPower(0);
        rightMotor2.setPower(0);
    }

    /**
     * Turn the robot clockwise
     * @deprecated This code doesn't work yet.  Edit it in the Autonomous class
     * @param degrees the number of degrees to turn <b>clockwise</b>
     */
    protected void turnClockwise(double degrees, double power) {
        //FIXME get this loop to refresh more frequently
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = angles.firstAngle, newAngle = angle + degrees;
        while (opModeIsActive() && Math.abs(angle - newAngle) > 0.5) {
            angle = angles.firstAngle;
            if (angle > newAngle) {
                rightMotor.setPower(power);
                rightMotor2.setPower(power);
                leftMotor.setPower(-power);
                leftMotor2.setPower(-power);
            } else {
                rightMotor.setPower(-power);
                rightMotor2.setPower(-power);
                leftMotor.setPower(power);
                leftMotor2.setPower(power);
            }
            //Update telemetry
            composeTelemetry();
            telemetry.addData("angle", angle);
            telemetry.addData("Degrees", degrees);
            telemetry.addData("Change", newAngle);
            telemetry.update();
        }
        //Stop motors
        rightMotor.setPower(0);
        rightMotor2.setPower(0);
        leftMotor.setPower(0);
        leftMotor2.setPower(0);
        telemetry.addData("Status", "Done turning");
        telemetry.update();
    }

    /**
     * Turn the robot clockwise
     * @deprecated This code doesn't work yet.  Edit it in the Autonomous class
     * @param degrees the number of degrees to turn <b>clockwise</b>
     */
    private void turnClockwise(double degrees) {
        turnClockwise(degrees, 0.2);
    }


    /**
     * Turn the robot a certain number of degrees in a certain direction
     * @param degrees
     * @param direction a string, either "Left" or "Right")
     */
    public void CarTurnDegreeDirection(double degrees, String direction) {

        double angle = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)), change = 0;

        if (direction.equals("Right")){
            change = angle - degrees;
        }
        if (direction.equals("Left")){
            change = angle + degrees;
        }
        if (change < -180) {
            change = change + 360;
        }
        if (change > 179.9) {
            change = change - 360;
        }
        while (Math.abs(change-angle)>2 && opModeIsActive()) {
            telemetry.addData("Angle", angle);
            telemetry.addData("Deg", degrees);

            telemetry.update();
            telemetry.addData("Change", change);
            angle = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            if (angle > change) {
                angle = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                rightMotor.setPower(-.1);
                rightMotor2.setPower(-.1);
                leftMotor.setPower(.1);
                leftMotor2.setPower(.1);
            }
            else {
                angle = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                rightMotor.setPower(.1);
                rightMotor2.setPower(.1);
                leftMotor.setPower(-.1);
                leftMotor2.setPower(-.1);
            }
        }

        rightMotor.setPower(0);
        rightMotor2.setPower(0);
        leftMotor.setPower(0);
        leftMotor2.setPower(0);
        telemetry.addData("A", "Return");
        telemetry.update();
    }


    /**
     * Runs the chassis motors at a given power for a certain number of inches
     * @Depreciated Use moveForwardInches()
     * @param speed the motor power
     * @param leftInches number of inches for the left motor to turn
     * @param rightInches number of inches for the right motor to turn
     * @param timeoutS the number of seconds before giving up
     */
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

    /**
     * Drive forward a certain distance based on encoder values
     * @param distance the distance to drive forward in <b>inches</b>.
     */
    public void driveForwardDistance(double distance) {
        int iterations = 0;
        long newLeftTarget = leftMotor2.getCurrentPosition() + (long) (distance * COUNTS_PER_INCH);
        long newRightTarget = rightMotor.getCurrentPosition() + (long) (distance * COUNTS_PER_INCH);
        boolean leftIsMoving = true, rightIsMoving = true;

        while ((leftIsMoving||rightIsMoving) && opModeIsActive()){
            telemetry.addData("Status", "Driving forward " + distance + " inches");
            telemetry.addData("Left Motor Position", leftMotor2.getCurrentPosition());
            telemetry.addData("Left Motor Speed", leftMotor.getPowerFloat());
            telemetry.addData("Right Motor Position", rightMotor.getCurrentPosition());
            telemetry.addData("Right Motor Speed", rightMotor.getPowerFloat());
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
                if(Math.abs(newRightTarget-rightMotor2.getCurrentPosition())<200){
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

    /**
     * Displays the red, green, and blue values on telemetry.
     * <b>Note: </b> this method does not call telemetry.update()
     *
     */
    protected void debugColorSensor(ColorSensor sensor){
        telemetry.addData("Red  ", blueSensorColor.red());
        telemetry.addData("Green", blueSensorColor.green());
        telemetry.addData("Blue ", blueSensorColor.blue());
    }
    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    protected void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = leftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = rightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftMotor.setTargetPosition(newLeftTarget);
            leftMotor2.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);
            rightMotor2.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftDrive(speed);
            rightDrive(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftDrive(leftSpeed);
                rightDrive(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            leftDrive(0);
            rightDrive(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        leftDrive(0);
        rightDrive(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftDrive(leftSpeed);
        rightDrive(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    }

