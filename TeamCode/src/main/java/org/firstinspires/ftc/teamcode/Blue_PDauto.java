
/*

Working:
Color Sensor (Some measuments need to be updated)
VuMark (distance traveled needs to be updated)
Gyro (needs to be tested)

Not working:
every thing in () up there
code needs to be cleaned
*/
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
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

//Import special FTC-related libraries
//Import Gyro


@Autonomous(name = "Blue_PDauto", group = "Linear Opmode")

public class Blue_PDauto extends LinearOpMode {

    //Direction variables
    public static int LEFT = 0, RIGHT = 1;
    //Initialize elapsed time object
    private ElapsedTime runtime = new ElapsedTime();

    //Initialize Gryo Vars

    // The IMU sensor object
    private BNO055IMU imu;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;

    //ROBOT HARDWARE
    //Instantiate chassis motors
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor leftMotor2;
    private DcMotor rightMotor2;
    //Instantiate servos
    private Servo color_servo;
    private Servo rotation_servo;
    //Instantiate sensors
    private ColorSensor sensorColor;
    private DistanceSensor sensorDistance;

    //Run this code when the "init" button is pressed
    @Override
    public void runOpMode() {
        //Give the OK message
        telemetry.addData("Status", "Initializing motors");
        telemetry.update();

        //Initialize robot hardware
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        leftMotor2 = hardwareMap.dcMotor.get("left_drive2");
        rightMotor2 = hardwareMap.dcMotor.get("right_drive2");
        color_servo = hardwareMap.get(Servo.class, "jewel_servo");
        rotation_servo = hardwareMap.get(Servo.class, "jewel_rotation_servo");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersV = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parametersV.vuforiaLicenseKey = "AfRvW7T/////AAAAGSZSz12Y5EPmopiqX9Cc17EX1TB9P/jfFGOFM0F4JDpeOV7v14oYyRCIzW09fiRPCvR2ivZcx3tHGuJTENamTxdwxZYSE72C9xuk7pmCjFeP5wfD3zF7bgYCrcVKk6Piahys8ccRRg93Dw4tC0kqkwrW5iz+u1x6+o6CctbGc8nxY3AzaH3W9HTU+QeGLv1xAx05YFOHwSgzNn9mZJYu2rYu2pBdKmb2Y918AlOwEC8QvbSARqI4aCKQle+Nplm/dBTFWO1p6sBIA8A7HHeb2vKcwHfkD10HEzDsZYuQNVxERAJ+7hfmmRLHLmtQJqXTWsQ6mMlKruGtm+s8m+4LhIEZy3dHnX4131J4bvSz1V6f";
        parametersV.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;


        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parametersV);
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        //Initilize Gryo

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parametersG = new BNO055IMU.Parameters();
        parametersG.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parametersG.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersG.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersG.loggingEnabled      = true;
        parametersG.loggingTag          = "IMU";
        parametersG.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersG);

        // Set up our telemetry dashboard
        composeTelemetry();

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Loop and update the dashboard
       /* while (opModeIsActive()) {
            telemetry.update();
        }*/

        telemetry.addData(">", "Press Play to start");
        telemetry.update();


        relicTrackables.activate();
        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000); // For Gyro Reset

        // Start the logging of measured acceleration -- Gyro


        telemetry.addData("Status", "Waiting for play button");
        telemetry.update();
        runtime.reset();


        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.update();



        color_servo.setPosition(1);
        rotation_servo.setPosition(.45);
        sleep(2000);


        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.update();
        telemetry.addData("Status", "Knocking Ball");
        telemetry.update();
        if (sensorColor.red() < sensorColor.blue() ) {  // is red // go froward knock red
            rotation_servo.setPosition(.2);

        }
        if (sensorColor.red() > sensorColor.blue() ) { // is blue // go back knock red
            rotation_servo.setPosition(.8);

        }

        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());

        telemetry.update();

        sleep(2000);
        color_servo.setPosition(.3);
        rotation_servo.setPosition(.45);

        telemetry.addData("Status", "VuMarking");
        telemetry.update();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "not visible");

        }
        else{
            telemetry.addData("VuMark", "%s visible", vuMark);



            switch(vuMark.ordinal()){
                case 1://Left

                    telemetry.addData("VuMark", "Left Action");
                    moveForward(0.75,.9);

                    break;
                case 2://Center

                    telemetry.addData("VuMark", "Center Action");
                    moveForward(0.5,.9);

                    break;
                case 3://Right

                    telemetry.addData("VuMark", "Right Action");
                    moveForward(0.25,.9);


                    break;
                default://Not visible

            }


        }

        telemetry.addData("Status", "Drove Forward now Turning");
        telemetry.update();


        //     CarTurnDegreeDirection(90,"Right");
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        moveForward(1,1);
        sleep(10000);
        turnClockwise(90.);


        //End of Autonomous
    }

    // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
     /*  leftMotor.setPower(.5);
        rightMotor.setPower(.5);
        leftMotor2.setPower(.5);
        rightMotor2.setPower(.5);
        runtime.reset(); */

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
        // Acquiring the angles is relatively expensive; we don't want
        // to do that in each of the three items that need that info, as that's
        // three times the necessary expense.
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();

        telemetry.addLine()
                .addData("status", (Func) () -> { return imu.getSystemStatus().toShortString();})
                .addData("calib", (Func) () -> {return imu.getCalibrationStatus().toString();});

        telemetry.addLine()
                .addData("heading", (Func) () -> {return formatAngle(angles.angleUnit, angles.firstAngle);})
                .addData("roll", (Func) () -> {return formatAngle(angles.angleUnit, angles.secondAngle);})
                .addData("pitch", (Func) () -> {return formatAngle(angles.angleUnit, angles.thirdAngle);});

        telemetry.addLine()
                .addData("grvty", (Func) () -> {return gravity.toString();})
                .addData("mag", (Func) () -> {
                    return String.format(Locale.getDefault(), "%.3f",
                            Math.sqrt(gravity.xAccel*gravity.xAccel+
                                    gravity.yAccel*gravity.yAccel
                                    + gravity.zAccel*gravity.zAccel));});
    }
    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    //Gyro End


    /**
     * Moves forward for a certain amount of time, then stops all chassis motors
     * @param time the amount of time to move foward <b>in seconds</b>
     * @param power the power of each of the motors
     */
    private void moveForward(double time, double power){
        long ms = (long) (time * 1000.0); //convert into miliseconds
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        leftMotor2.setPower(power);
        rightMotor2.setPower(power);
        sleep(ms);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor2.setPower(0);
        rightMotor2.setPower(0);

    }

    /**
     * Turns the robot a certain amount of degrees, based on the built in gyroscope
     * @param degrees the amount of degrees to turn clockwise.  If the robot must turn
     *                counterclockwise, use a negative number
     * @param threshold
     */
    public void turnClockwise(double degrees, double threshold){
        //Initialize angle and change variables
        double angle = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)),newAngle;
        newAngle = angle + degrees;

        while(Math.abs(newAngle-angle)<threshold){
            if(angle<newAngle){
                rightMotor.setPower(-.5);
                rightMotor2.setPower(-.5);
                leftMotor.setPower(.5);
                leftMotor2.setPower(.5);
            }
            else{
                rightMotor.setPower(.5);
                rightMotor2.setPower(.5);
                leftMotor.setPower(-.5);
                leftMotor2.setPower(-.5);
            }

            //Update angles
            angle = angles.firstAngle;
        }
    }
    public void turnClockwise(double deg){
        turnClockwise(deg, 1.0);
    }
}





