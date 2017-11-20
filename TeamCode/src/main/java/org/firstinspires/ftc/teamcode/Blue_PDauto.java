
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
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import java.util.concurrent.TimeUnit;

//Import special FTC-related libraries
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

//Import Gyro 

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name = "Blue_PDauto", group = "Linear Opmode")

public class Blue_PDauto extends LinearOpMode {
    //Initialize and instantiate vuforia variables
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    //Initialize elapsed time object
    private ElapsedTime runtime = new ElapsedTime();

    //Initialize Gryo Vars (I think - PD)

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;




    //ROBOT HARDWARE
    //Instantiate chassis motors
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor leftMotor2;
    DcMotor rightMotor2;
    //Instantiate servos
    Servo color_servo;
    Servo glyph_servo;
    Servo rotation_servo;
    //Instantiate sensors
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

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


        this.vuforia = ClassFactory.createVuforiaLocalizer(parametersV);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
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
                    CarSecPow(0.75,.9);

                    break;
                case 2://Center

                    telemetry.addData("VuMark", "Center Action");
                    CarSecPow(0.5,.9);

                    break;
                case 3://Right

                    telemetry.addData("VuMark", "Right Action");
                    CarSecPow(0.25,.9);


                    break;
                default://Not visible

            }


        }

        telemetry.addData("Status", "Drove Forward now Turning");
        telemetry.update();


        //     CarTurnDegreeDirection(90,"Right");
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        CarSecPow(1,1);
        sleep(10000);
        CarTurnDegreeDirection(90,"Left");


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

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })

                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
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

    // Makes it go for x seconds

    public void CarSecPow(double sec, double pow){
        double mili = (double) sec * 1000.0; //convert into miliseconds
        leftMotor.setPower(pow);
        rightMotor.setPower(pow);
        leftMotor2.setPower(pow);
        rightMotor2.setPower(pow);
        sleep((int) mili);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor2.setPower(0);
        rightMotor2.setPower(0);

    }
    double Angle = 0;
    double change = 0;
    public void CarTurnDegreeDirection(double deg,String dir){

        Angle = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        if(dir == "Right") {
            change = Angle - deg;

        }
        if(dir == "Left") {
            change = Angle + deg;

        }
        if (change < -180){
            change = change +360;
        }
        if (change > 179.9){
            change = change -360;
        }
        Angle = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        while (Angle - 1 > change ) {
            Angle = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            rightMotor.setPower(-.5);
            rightMotor2.setPower(-.5);
            leftMotor.setPower(.5);
            leftMotor2.setPower(.5);
            telemetry.addData("Angle", Angle );
            telemetry.addData("Deg", deg);
            telemetry.addData("Change", change);
            telemetry.addData("Turn Right", "Right");
            telemetry.update();
        }

        while (Angle + 1 < change) {
            Angle = Double.para seDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            rightMotor.setPower(.5);
            rightMotor2.setPower(.5);
            leftMotor.setPower(-.5);
            leftMotor2.setPower(-.5);
            telemetry.addData("Angle", Angle );
            telemetry.addData("Deg", deg);
            telemetry.addData("Change", change);
            telemetry.addData("Turn Left", "Left");
            telemetry.update();
        }
    }


}





