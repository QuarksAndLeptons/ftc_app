package org.firstinspires.ftc.teamcode;
//Import standard FTC libraries

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Locale;
//Import hardware libraries
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
//Import Gyro
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

//Import special FTC-related libraries
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

//Import navigation libraries
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name = "Blue_Autonomous", group = "Linear Opmode")
public class Blue_Autonomous extends LinearOpMode {
    //Initialize and instantiate vuforia variables
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    //Initialize elapsed time object 
    private ElapsedTime runtime = new ElapsedTime();

    // The IMU sensor object
    private BNO055IMU imu;

    // Orientation and acceleration variables from the built in 9-axis accelerometer
    private Orientation angles;
    private Acceleration gravity;

    //ROBOT HARDWARE
    //Instantiate chassis motors
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor leftMotor2;
    private DcMotor rightMotor2;
    private DcMotor dropMotor;
    //Instantiate servos
    private Servo color_servo;
    private Servo rotation_servo;
    private Servo color_servo2;
    private Servo rotation_servo2;
    //Instantiate sensors
    private ColorSensor sensorColor;
    private DistanceSensor sensorDistance;

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
        //Initialize the servos
        color_servo = hardwareMap.get(Servo.class, "jewel_servo");
        rotation_servo = hardwareMap.get(Servo.class, "jewel_rotation_servo");
        //Left Side
        color_servo2 = hardwareMap.get(Servo.class, "jewel_servo2");
        rotation_servo2 = hardwareMap.get(Servo.class, "jewel_rotation_servo2");
        //Finally initialize the sensors
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        //Initialize Vuforia extension
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersV = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parametersV.vuforiaLicenseKey = "AfRvW7T/////AAAAGSZSz12Y5EPmopiqX9Cc17EX1TB9P/jfFGOFM0F4JDpeOV7v14oYyRCIzW09fiRPCvR2ivZcx3tHGuJTENamTxdwxZYSE72C9xuk7pmCjFeP5wfD3zF7bgYCrcVKk6Piahys8ccRRg93Dw4tC0kqkwrW5iz+u1x6+o6CctbGc8nxY3AzaH3W9HTU+QeGLv1xAx05YFOHwSgzNn9mZJYu2rYu2pBdKmb2Y918AlOwEC8QvbSARqI4aCKQle+Nplm/dBTFWO1p6sBIA8A7HHeb2vKcwHfkD10HEzDsZYuQNVxERAJ+7hfmmRLHLmtQJqXTWsQ6mMlKruGtm+s8m+4LhIEZy3dHnX4131J4bvSz1V6f";
        parametersV.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; //Set camera
        vuforia = ClassFactory.createVuforiaLocalizer(parametersV);
        //Get the assets for Vuforia
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Do some calibration and activation
        // Calibrate Gyro
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        //Activate Vuforia
        relicTrackables.activate();
        //Reset the runtime.
        runtime.reset();

        // Move the color sensor to the jewels
        telemetry.addData("Status", "Reading color of jewel");
        telemetry.update();
        color_servo.setPosition(.9);
        rotation_servo.setPosition(.45);
        sleep(2000);

        //Knock over the blue ball
        if (sensorColor.red() < sensorColor.blue()) {  // is red // go froward knock red
            rotation_servo.setPosition(.2);
            telemetry.addData("Status", "detected BLUE (moving servo FORWARD)");
            telemetry.update();
        } else if (sensorColor.red() > sensorColor.blue()) { // is blue // go back knock red
            rotation_servo.setPosition(.8);
            telemetry.addData("Status", "detected RED (moving servo FORWARD)");
            telemetry.update();
        }
        telemetry.update();

        //Wait for the servo to finish its movement
        sleep(2000);
        //Reset the servos to retract the sensor
        color_servo.setPosition(.3);
        rotation_servo.setPosition(.45);

        //For now we are only driving forward
        telemetry.addData("Status", "Moving forward");
        telemetry.update();
        moveForward(1.2,-1.0);
        telemetry.addData("Status", "Done");
        telemetry.update();
        /*
        //Detect the Vumark
        telemetry.addData("Status", "VuMarking");
        telemetry.update();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        sleep(2000);
        //If Vumark not visible
        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "not visible");
            telemetry.update();
            encoderDrive(-.5, -21, -21, 10);
            CarTurnDegreeDirection(45, "Right");
            telemetry.addData("Status", "Successful");
            telemetry.update();
            encoderDrive(-.5, -10, -10, 10);  //(power,left inches, right inches, timeout)
            dropMotor.setTargetPosition(400);
            dropMotor.setPower(-0.25);
        } else {
            telemetry.addData("VuMark", "%s visible", vuMark);
            telemetry.update();
            //Choose what to do based on the Vumark the camera sees
            switch (vuMark.ordinal()) {
                case 1://Left
                    telemetry.addData("VuMark", "Left Action");
                    encoderDrive(-.3, -21, -21, 10);
                    turnClockwise(45);
                    telemetry.addData("Status", "Successful");
                    telemetry.update();
                    encoderDrive(-.3, -10, -10, 10);  //(power,left inches, right inches, timeout)
                    dropMotor.setTargetPosition(400);
                    dropMotor.setPower(-0.25);
                    break;

                case 2://Center
                    telemetry.addData("VuMark", "Center Action");
                    encoderDrive(.3, 23, 23, 10);
                    turnClockwise(30);
                    dropMotor.setTargetPosition(400);
                    dropMotor.setPower(-0.25);
                    encoderDrive(.2, 2, 2, 10);
                    break;

                case 3://Right
                    telemetry.addData("VuMark", "Right Action");
                    //encoderDrive(-.3,-24,-24,10);
                    //FIXME
                    break;
            }

        }

        //Autonomous code is finished
        telemetry.addData("Status", "Done!");
        telemetry.update();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        sleep(1000);
        */
        //End of Autonomous
    }


    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    // Gyro
    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------
    private void composeTelemetry() {
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
     * Turn the robot clockwise
     *
     * @param degrees the number of degrees to turn <b>clockwise</b>
     */
    private void turnClockwise(double degrees, double power) {
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

    private void turnClockwise(double degrees) {
        turnClockwise(degrees, 0.2);
    }


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
     * Drive forward a certain distance based on encoder values
     * @param distance the distance to drive forward in <b>inches</b>.
     */
    public void driveForward(double distance) {
        long newLeftTarget = leftMotor.getCurrentPosition() + (long) (distance * COUNTS_PER_INCH);
        long newRightTarget = rightMotor2.getCurrentPosition() + (long) (distance * COUNTS_PER_INCH);
        boolean leftIsMoving = true, rightIsMoving = true;

        while ((leftIsMoving||rightIsMoving) && opModeIsActive()){
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
                else if (rightMotor.getCurrentPosition() < newRightTarget) {
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







