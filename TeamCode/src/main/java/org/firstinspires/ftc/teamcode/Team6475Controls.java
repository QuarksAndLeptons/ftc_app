package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
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

import static java.lang.Math.abs;

/**
 * Created by Micah Mundy on 12/5/17.
 */

public abstract class Team6475Controls extends LinearOpMode {
    //Initialize and instantiate vuforia variables
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    //Initialize elapsed time object
    protected ElapsedTime runtime = new ElapsedTime();

    // The IMU sensor object
    protected BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double absoluteAngle, power = .30, correction;
    PIDController pidRotate, pidDrive;

    // Orientation and acceleration variables from the built in 9-axis accelerometer
    protected Orientation angles;
    protected Acceleration gravity;

    //ROBOT HARDWARE
    //Instantiate chassis motors

    protected DcMotorEx leftMotor;
    protected DcMotorEx rightMotor;
    protected DcMotorEx leftMotor2;
    protected DcMotorEx rightMotor2;
    protected DcMotorEx relicExtendMotor;
    protected DcMotorEx relicRetractMotor;
    protected DcMotor glyphLiftMotor;

    //Instantiate servos
    protected Servo blueColorServo;
    protected Servo jewelRotationServo;
    protected Servo glyphTopLeft, glyphTopRight, glyphBottomLeft, glyphBottomRight;
    protected Servo relicClaw;
    protected Servo relicLift;

    //Instantiate sensors
    ColorSensor blueSensorColor;

    //Initlize encoder variables
    protected double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Encoder
    protected double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    protected double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    protected double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = .3;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = .3;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 2.5;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = .005;     // .02 Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = .010;     // .05 Larger is more responsive, but also less stable




    //Initialize Vuforia variables
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    /**
     * Initializes all of the motors and servos.
     * <b>Be sure to call this before waitForStart()</b>
     */
    protected void initializeHardware() {
        //Give the OK message
        telemetry.addData("Status", "Initializing hardware");
        telemetry.update();

        //Initialize robot hardware
        //Begin with the chassis
        leftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftFront");
        rightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightFront");
        leftMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftRear");
        rightMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightRear");
        relicExtendMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "relicExtendMotor");
        relicRetractMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "relicRetractMotor");
        glyphLiftMotor = hardwareMap.get(DcMotor.class, "glyphlift");

        //Reset the encoders on the chassis to 0
        leftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set the motor modes
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Reverse the right motors so all motors move forward when set to a positive speed.
        rightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Initialize glyph lifting servos
        glyphTopLeft = hardwareMap.get(Servo.class, "glyphTopLeft");
        glyphTopRight = hardwareMap.get(Servo.class, "glyphTopRight");
        glyphBottomLeft = hardwareMap.get(Servo.class, "glyphBottomLeft");
        glyphBottomRight = hardwareMap.get(Servo.class, "glyphBottomRight");

        //Initialize the servos
       blueColorServo = hardwareMap.get(Servo.class, "jewelServo");
       jewelRotationServo = hardwareMap.get(Servo.class, "jewelRotationServo");
        relicClaw = hardwareMap.get(Servo.class, "relicClaw");
        relicLift = hardwareMap.get(Servo.class, "relicLift");


        //Initialize sensors
        blueSensorColor = hardwareMap.get(ColorSensor.class, "BlueColorSensor");


        //Initialize Vuforia extension
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersV = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parametersV.vuforiaLicenseKey = "AfRvW7T/////AAAAGSZSz12Y5EPmopiqX9Cc17EX1TB9P/jfFGOFM0F4JDpeOV7v14oYyRCIzW09fiRPCvR2ivZcx3tHGuJTENamTxdwxZYSE72C9xuk7pmCjFeP5wfD3zF7bgYCrcVKk6Piahys8ccRRg93Dw4tC0kqkwrW5iz+u1x6+o6CctbGc8nxY3AzaH3W9HTU+QeGLv1xAx05YFOHwSgzNn9mZJYu2rYu2pBdKmb2Y918AlOwEC8QvbSARqI4aCKQle+Nplm/dBTFWO1p6sBIA8A7HHeb2vKcwHfkD10HEzDsZYuQNVxERAJ+7hfmmRLHLmtQJqXTWsQ6mMlKruGtm+s8m+4LhIEZy3dHnX4131J4bvSz1V6f";
        parametersV.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; //Set camera
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


        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        pidRotate = new PIDController(.005, 0, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.015, 0, .015); //Oscillation starts at Kp=.030

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        // Set up our telemetry dashboard
        composeTelemetry();
        // Set the g

    }

    /**
     * Begin Vuforia tracking and caliberate gyro.
     * <b>Call this <i>after</i> waitForStart()</b>
     */
    protected void startAdvancedSensing() {
        //Do some calibration and activation
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 40);
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
        telemetry.addLine().addData("status", () -> imu.getSystemStatus().toShortString()).addData("calib", () -> imu.getCalibrationStatus().toString());
        telemetry.addLine().addData("heading", () -> formatAngle(angles.angleUnit, angles.firstAngle)).addData("roll", () -> formatAngle(angles.angleUnit, angles.secondAngle)).addData("pitch", () -> formatAngle(angles.angleUnit, angles.thirdAngle));
        telemetry.addLine().addData("grvty", () -> gravity.toString())

                .addData("mag", () -> String.format(Locale.getDefault(), "%.3f", Math.sqrt(gravity.xAccel * gravity.xAccel + gravity.yAccel * gravity.yAccel + gravity.zAccel * gravity.zAccel)));
    }

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));

    }

    private String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /**
     * Sets the power of both right motors
     *
     * @param power the power of each of the motors
     */
    protected void rightDrive(double power) {
        rightMotor.setPower(power);
        rightMotor2.setPower(power);
    }

    /**
     * Sets the power of both left motors
     *
     * @param power the power of each of the motors
     */
    protected void leftDrive(double power) {
        leftMotor.setPower(power);
        leftMotor2.setPower(power);

    }

    /**
     * Displays the red, green, and blue values on telemetry.
     * <b>Note: </b> this method does not call telemetry.update()
     */
    protected void debugColorSensor(ColorSensor sensor) {
        telemetry.addData("Red  ", blueSensorColor.red());
        telemetry.addData("Green", blueSensorColor.green());
        telemetry.addData("Blue ", blueSensorColor.blue());
    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param timeout  the number of seconds to take control of the autonomous program
     *                 before giving up
     */
    protected void gyroDrive(double speed, double distance, double angle, double timeout) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = leftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = rightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftMotor.setTargetPosition(newLeftTarget);
            leftMotor2.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);
            rightMotor2.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            leftMotor.setTargetPositionTolerance(100);
            rightMotor.setTargetPositionTolerance(100);
            // start motion.
            speed = Range.clip(speed, -1.0, 1.0);
            leftDrive(speed);
            rightDrive(speed);


            double timeoutTime = runtime.seconds() + timeout;
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (leftMotor.isBusy() && rightMotor.isBusy()) && runtime.seconds() <= timeoutTime) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;
                telemetry.addData("left: ", leftSpeed);
                telemetry.addData("right", rightSpeed);
                telemetry.update();
                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(abs(leftSpeed), abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftDrive(leftSpeed);
                rightDrive(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            leftDrive(0);
            rightDrive(0);
        }
    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    protected void gyroDrive(double speed, double distance, double angle) {
        gyroDrive(speed, distance, angle, 60);
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {
        gyroTurn(speed, angle, 60);
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed   Desired speed of turn.
     * @param angle   Absolute Angle (in Degrees) relative to last gyro reset.
     *                0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                If a relative angle is required, add/subtract from current heading.
     * @param timeout the number of seconds to take control of the autonomous program
     *                before giving up
     */
    public void gyroTurn(double speed, double angle, double timeout) {
        //Ensure the motors are in the right configuration
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double timeoutTime = runtime.seconds() + timeout;
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF) && runtime.seconds() < timeoutTime) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

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
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    private boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = Range.clip(speed * steer,-1,1);
            leftSpeed = -rightSpeed;
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
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    private double getSteer(double error, double PCoeff) {
        return Range.clip(PCoeff * error, -1, 1);


    }

     /**
     * Grab the glyphs
     */
    protected void grabGlyphs() {
        //FIXED
        glyphTopLeft.setPosition(.24);
        glyphTopRight.setPosition(.68);
        glyphBottomLeft.setPosition(0.68);
        glyphBottomRight.setPosition(0.44);
    }

    /**
     * Release the glyphs CLAWS Straight
     */
    protected void releaseGlyphs() {
        //FIXED
        glyphTopLeft.setPosition(0.0);
        glyphTopRight.setPosition(1.0);
        glyphBottomLeft.setPosition(0.92);
        glyphBottomRight.setPosition(.18);
    }



    /**
     * Sets the speed of the lifting mechanism.
     * NOTE: the glyph lifting motor behaves like a continuous rotation servo, so this method
     * converts this value to the servo equivalent.
     *
     * @param speed the speed of the glyph-lifting mechanism, where -1.0 is the maximum
     *              downward speed, and +1.0 is the maximum upward speed.
     **/
    protected void liftGlyphs(double speed) {
        glyphLiftMotor.setPower(speed);
    }


    //END GAME CONTROLS
    /**
     * Grabs the relic
     *
     **/
    protected void grabRelic(){
        relicClaw.setPosition(0);

    }

    /**
     * Releases the relic
     */
    protected void releaseRelic(){
        relicClaw.setPosition(.9);


    }

    /**
     * Sets the power of the relic motor to extend the "relic arm"
     * @param power the power of the relic motor
     */
    protected void deployRelic(double power){
        if(power>0.0){
            relicExtendMotor.setPower(power);
            relicRetractMotor.setPower(-0.1);
        }
        else{
            relicExtendMotor.setPower(-0.4);
            relicRetractMotor.setPower(power);
        }

    }

    /**
     * Lifts the relic.  Ideal for endgame where the relic must
     * go over the field wall and throwing the relic, if necessary
     */
    protected void liftRelic(){
        relicLift.setPosition(.9);

    }

    /**
     * Drops the relic.  Ideal for initializing servo motors and
     * lowering the glyph after carrying it over the field wall
     */
    protected void dropRelic(){
        relicLift.setPosition(.1);

    }


    // Set up parameters for driving in a straight line.

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param power    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param timeout  the number of seconds to take control of the autonomous program
     *                 before giving up
     */

    protected void drive(double power, double distance, double angle, double timeout) {
        //Ensure the motors are in the right configuration
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double leftPower;
        double rightPower;

        pidDrive.setSetpoint(angle);
        pidDrive.setOutputRange(-power, power);
        pidDrive.setInputRange(-179, 180);
        pidDrive.enable();

        // drive until end of period.

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = leftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = rightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftMotor.setTargetPosition(newLeftTarget);
            leftMotor2.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);
            rightMotor2.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            leftMotor.setTargetPositionTolerance(100);
            rightMotor.setTargetPositionTolerance(100);

            // Use PID with imu input to drive in a straight line.

            double timeoutTime = runtime.seconds() + timeout;
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (leftMotor.isBusy() && rightMotor.isBusy()) && runtime.seconds() <= timeoutTime) {
                correction = pidDrive.performPID(getAngle());

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) correction *= -1.0;

                leftPower = power - correction;
                rightPower = power + correction;

                // set power levels.

                leftDrive(leftPower);
                rightDrive(rightPower);

                telemetry.addData("1 imu heading", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("2 global heading", absoluteAngle);
                telemetry.addData("3 correction", correction);
                // Display drive status for the driver.
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftPower, rightPower);
                telemetry.update();
                telemetry.update();



                // Stop all motion;
                leftDrive(0);
                rightDrive(0);

            }
        }
    }




    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;


        if (deltaAngle < -180) deltaAngle += 360;
        else if (deltaAngle > 180) deltaAngle -= 360;

        lastAngles = angles;

        absoluteAngle += deltaAngle;

        return absoluteAngle;

    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    protected void turnToHeading(int degrees, double power) {
        // restart imu angle tracking.
        //resetAngle();

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle with a minimum of 20%.
        // This is to prevent the robots momentum from overshooting the turn after we turn off the
        // power. The PID controller reports onTarget() = true when the difference between turn
        // angle and target angle is within 2% of target (tolerance). This helps prevent overshoot.
        // The minimum power is determined by testing and must enough to prevent motor stall and
        // complete the turn. Note: if the gap between the starting power and the stall (minimum)
        // power is small, overshoot may still occur. Overshoot is dependant on the motor and
        // gearing configuration, starting power, weight of the robot and the on target tolerance.
        //Ensure the motors are in the right configuration
        //Reset the encoders on the chassis to 0
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(-179, 180);
        pidRotate.setOutputRange(.1, power);
        pidRotate.setTolerance(2);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                leftDrive(power);
                rightDrive(-power);
                sleep(100);
            }

            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                leftDrive(-power);
                rightDrive(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        } else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                leftDrive(-power);
                rightDrive(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        telemetry.addData("1 imu heading", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("2 global heading", absoluteAngle);
        telemetry.addData("3 correction", correction);
        telemetry.update();

        // turn the motors off.
        rightDrive(0);
        leftDrive(0);

        // wait for rotation to stop.
        sleep(500);


    }



    public class PIDController {
        private double m_P;                                 // factor for "proportional" control
        private double m_I;                                 // factor for "integral" control
        private double m_D;                                 // factor for "derivative" control
        private double m_input;                 // sensor input for pid controller
        private double m_maximumOutput = 1.0;   // |maximum output|
        private double m_minimumOutput = -1.0;  // |minimum output|
        private double m_maximumInput = 0.0;    // maximum input - limit setpoint to this
        private double m_minimumInput = 0.0;    // minimum input - limit setpoint to this
        private boolean m_continuous = false;   // do the endpoints wrap around? eg. Absolute encoder
        private boolean m_enabled = false;              //is the pid controller enabled
        private double m_prevError = 0.0;           // the prior sensor input (used to compute velocity)
        private double m_totalError = 0.0;      //the sum of the errors for use in the integral calc
        private double m_tolerance = 0.02;          //the percentage error that is considered on target
        private double m_setpoint = 0.0;
        private double m_error = 0.0;
        private double m_result = 0.0;

        /**
         * Allocate a PID object with the given constants for P, I, D
         *
         * @param Kp the proportional coefficient
         * @param Ki the integral coefficient
         * @param Kd the derivative coefficient
         */
        public PIDController(double Kp, double Ki, double Kd) {
            m_P = Kp;
            m_I = Ki;
            m_D = Kd;
        }

        /**
         * Read the input, calculate the output accordingly, and write to the output.
         * This should only be called by the PIDTask
         * and is created during initialization.
         */
        private void calculate() {
            int sign = 1;

            // If enabled then proceed into controller calculations
            if (m_enabled) {
                // Calculate the error signal
                m_error = m_setpoint - m_input;

                // If continuous is set to true allow wrap around
                if (m_continuous) {
                    if (Math.abs(m_error) > (m_maximumInput - m_minimumInput) / 2) {
                        if (m_error > 0) m_error = m_error - m_maximumInput + m_minimumInput;
                        else m_error = m_error + m_maximumInput - m_minimumInput;
                    }
                }

                // Integrate the errors as long as the upcoming integrator does
                // not exceed the minimum and maximum output thresholds.

                if ((Math.abs(m_totalError + m_error) * m_I < m_maximumOutput) && (Math.abs(m_totalError + m_error) * m_I > m_minimumOutput))
                    m_totalError += m_error;

                // Perform the primary PID calculation
                m_result = m_P * m_error + m_I * m_totalError + m_D * (m_error - m_prevError);

                // Set the current error to the previous error for the next cycle.
                m_prevError = m_error;

                if (m_result < 0) sign = -1;    // Record sign of result.

                // Make sure the final result is within bounds. If we constrain the result, we make
                // sure the sign of the constrained result matches the original result sign.
                if (Math.abs(m_result) > m_maximumOutput) m_result = m_maximumOutput * sign;
                else if (Math.abs(m_result) < m_minimumOutput) m_result = m_minimumOutput * sign;
            }
        }

        /**
         * Set the PID Controller gain parameters.
         * Set the proportional, integral, and differential coefficients.
         *
         * @param p Proportional coefficient
         * @param i Integral coefficient
         * @param d Differential coefficient
         */
        public void setPID(double p, double i, double d) {
            m_P = p;
            m_I = i;
            m_D = d;
        }

        /**
         * Get the Proportional coefficient
         *
         * @return proportional coefficient
         */
        public double getP() {
            return m_P;
        }

        /**
         * Get the Integral coefficient
         *
         * @return integral coefficient
         */
        public double getI() {
            return m_I;
        }

        /**
         * Get the Differential coefficient
         *
         * @return differential coefficient
         */
        public double getD() {
            return m_D;
        }

        /**
         * Return the current PID result for the last input set with setInput().
         * This is always centered on zero and constrained the the max and min outs
         *
         * @return the latest calculated output
         */
        public double performPID() {
            calculate();
            return m_result;
        }

        /**
         * Return the current PID result for the specified input.
         *
         * @param input The input value to be used to calculate the PID result.
         *              This is always centered on zero and constrained the the max and min outs
         * @return the latest calculated output
         */
        public double performPID(double input) {
            setInput(input);
            return performPID();
        }

        /**
         * Set the PID controller to consider the input to be continuous,
         * Rather then using the max and min in as constraints, it considers them to
         * be the same point and automatically calculates the shortest route to
         * the setpoint.
         *
         * @param continuous Set to true turns on continuous, false turns off continuous
         */
        public void setContinuous(boolean continuous) {
            m_continuous = continuous;
        }

        /**
         * Set the PID controller to consider the input to be continuous,
         * Rather then using the max and min in as constraints, it considers them to
         * be the same point and automatically calculates the shortest route to
         * the setpoint.
         */
        public void setContinuous() {
            this.setContinuous(true);
        }

        /**
         * Sets the maximum and minimum values expected from the input.
         *
         * @param minimumInput the minimum value expected from the input
         * @param maximumInput the maximum value expected from the output
         */
        public void setInputRange(double minimumInput, double maximumInput) {
            m_minimumInput = (minimumInput);
            m_maximumInput = (maximumInput);
            setSetpoint(m_setpoint);
        }

        /**
         * Sets the minimum and maximum values to write.
         *
         * @param minimumOutput the minimum value to write to the output, always positive
         * @param maximumOutput the maximum value to write to the output, always positive
         */
        public void setOutputRange(double minimumOutput, double maximumOutput) {
            m_minimumOutput = Math.abs(minimumOutput);
            m_maximumOutput = Math.abs(maximumOutput);
        }

        /**
         * Set the setpoint for the PIDController
         *
         * @param setpoint the desired setpoint
         */
        public void setSetpoint(double setpoint) {
            int sign = 1;

            if (m_maximumInput > m_minimumInput) {
                if (setpoint < 0) sign = -1;

                if (Math.abs(setpoint) > m_maximumInput) m_setpoint = m_maximumInput * sign;
                else if (Math.abs(setpoint) < m_minimumInput) m_setpoint = m_minimumInput * sign;
                else m_setpoint = setpoint;
            } else m_setpoint = setpoint;
        }

        /**
         * Returns the current setpoint of the PIDController
         *
         * @return the current setpoint
         */
        public double getSetpoint() {
            return m_setpoint;
        }

        /**
         * Retruns the current difference of the input from the setpoint
         *
         * @return the current error
         */
        public synchronized double getError() {
            return m_error;
        }

        /**
         * Set the percentage error which is considered tolerable for use with
         * OnTarget. (Input of 15.0 = 15 percent)
         *
         * @param percent error which is tolerable
         */
        public void setTolerance(double percent) {
            m_tolerance = percent;
        }

        /**
         * Return true if the error is within the percentage of the total input range,
         * determined by setTolerance. This assumes that the maximum and minimum input
         * were set using setInputRange.
         *
         * @return true if the error is less than the tolerance
         */
        public boolean onTarget() {
            return (Math.abs(m_error) < Math.abs(m_tolerance / 100 * (m_maximumInput - m_minimumInput)));
        }

        /**
         * Begin running the PIDController
         */
        public void enable() {
            m_enabled = true;
        }

        /**
         * Stop running the PIDController.
         */
        public void disable() {
            m_enabled = false;
        }

        /**
         * Reset the previous error,, the integral term, and disable the controller.
         */
        public void reset() {
            disable();
            m_prevError = 0;
            m_totalError = 0;
            m_result = 0;
        }

        /**
         * Set the input value to be used by the next call to performPID().
         *
         * @param input Input value to the PID calculation.
         */
        public void setInput(double input) {
            int sign = 1;

            if (m_maximumInput > m_minimumInput) {
                if (input < 0) sign = -1;

                if (Math.abs(input) > m_maximumInput) m_input = m_maximumInput * sign;
                else if (Math.abs(input) < m_minimumInput) m_input = m_minimumInput * sign;
                else m_input = input;
            } else m_input = input;
        }
    }
    /**
     * Set the input value for integrated PID.
     *
     * @param NEW_P Input value to the PID calculation.
     * @param NEW_I
     * @param NEW_D
     *
     */
    protected void setPID(double NEW_P, double NEW_I, double NEW_D) {
        /**
         * Created by tom on 9/26/17.
         * This assumes that you are using a REV Robotics Expansion Hub
         * as your DC motor controller. This op mode uses the extended/enhanced
         * PID-related functions of the DcMotorControllerEx class.
         * The REV Robotics Expansion Hub supports the extended motor controller
         * functions, but other controllers (such as the Modern Robotics and
         * Hitechnic DC Motor Controllers) do not.
         */

        // get a reference to the motor controller and cast it as an extended functionality controller.
        // we assume it's a REV Robotics Expansion Hub (which supports the extended controller functions).
        DcMotorControllerEx lmotorControllerEx = (DcMotorControllerEx) leftMotor.getController();
        DcMotorControllerEx rmotorControllerEx = (DcMotorControllerEx) rightMotor.getController();

        // get the port number of our configured motor.
        int lmotorIndex = leftMotor.getPortNumber();
        int rmotorIndex = rightMotor.getPortNumber();

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDCoefficients lpidOrig = lmotorControllerEx.getPIDCoefficients(lmotorIndex, DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients rpidOrig = rmotorControllerEx.getPIDCoefficients(rmotorIndex, DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients.
        PIDCoefficients lpidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        PIDCoefficients rpidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        lmotorControllerEx.setPIDCoefficients(lmotorIndex, DcMotor.RunMode.RUN_USING_ENCODER, lpidNew);
        rmotorControllerEx.setPIDCoefficients(rmotorIndex, DcMotor.RunMode.RUN_USING_ENCODER, rpidNew);

        // re-read coefficients and verify change.
        PIDCoefficients lpidModified = lmotorControllerEx.getPIDCoefficients(lmotorIndex, DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients rpidModified = lmotorControllerEx.getPIDCoefficients(rmotorIndex, DcMotor.RunMode.RUN_USING_ENCODER);

        // display info to user.
        while (opModeIsActive())

        {
            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.addData("Left P,I,D (orig)", "%.04f, %.04f, %.0f", lpidOrig.p, lpidOrig.i, lpidOrig.d);
            telemetry.addData("Right P,I,D (orig)", "%.04f, %.04f, %.0f", rpidOrig.p, rpidOrig.i, rpidOrig.d);
            telemetry.addData("Left P,I,D (modified)", "%.04f, %.04f, %.04f", lpidModified.p, lpidModified.i, lpidModified.d);
            telemetry.addData("Right P,I,D (modified)", "%.04f, %.04f, %.04f", rpidModified.p, rpidModified.i, rpidModified.d);

            telemetry.update();

        }
    }
}
