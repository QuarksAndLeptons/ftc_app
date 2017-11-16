package org.firstinspires.ftc.teamcode;


//Import standard FTC libraries
import android.graphics.Color;
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


@Autonomous(name = "BlueWithVM_PDauto", group = "Linear Opmode")

public class BlueWithVM_PDauto extends LinearOpMode {
    //Initialize and instantiate vuforia variables
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    //Initialize elapsed time object
    private ElapsedTime runtime = new ElapsedTime();
    
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
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
         parameters.vuforiaLicenseKey = "AfRvW7T/////AAAAGSZSz12Y5EPmopiqX9Cc17EX1TB9P/jfFGOFM0F4JDpeOV7v14oYyRCIzW09fiRPCvR2ivZcx3tHGuJTENamTxdwxZYSE72C9xuk7pmCjFeP5wfD3zF7bgYCrcVKk6Piahys8ccRRg93Dw4tC0kqkwrW5iz+u1x6+o6CctbGc8nxY3AzaH3W9HTU+QeGLv1xAx05YFOHwSgzNn9mZJYu2rYu2pBdKmb2Y918AlOwEC8QvbSARqI4aCKQle+Nplm/dBTFWO1p6sBIA8A7HHeb2vKcwHfkD10HEzDsZYuQNVxERAJ+7hfmmRLHLmtQJqXTWsQ6mMlKruGtm+s8m+4LhIEZy3dHnX4131J4bvSz1V6f";
         parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
         
         
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();
        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Status", "Waiting for play button");
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
             
        if (sensorColor.red() < sensorColor.blue() ) {  // is red // go froward knock red
                    rotation_servo.setPosition(.2);
                
        } 
        if (sensorColor.red() > sensorColor.blue() ) { // not red // go back knock red
                    rotation_servo.setPosition(.8);         
                    
                    } 
                    
                    telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.update();
            
        sleep(2000);        
        color_servo.setPosition(.3);
        rotation_servo.setPosition(.45); 
        
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
           
        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "not visible");
            
        }
        else{
            telemetry.addData("VuMark", "%s visible", vuMark);
            
           
            
            switch(vuMark.ordinal()){
                case 1://Left
                    
            leftMotor.setPower(.5);
            rightMotor.setPower(.5);
            leftMotor2.setPower(.5);
            rightMotor2.setPower(.5);
            sleep(750);   
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            leftMotor2.setPower(0);
            rightMotor2.setPower(0);
                    
                    break;
                case 2://Center
            leftMotor.setPower(.5);
            rightMotor.setPower(.5);
            leftMotor2.setPower(.5);
            rightMotor2.setPower(.5);
            sleep(500);   
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            leftMotor2.setPower(0);
            rightMotor2.setPower(0);
                    
                    break;
                case 3://Right
            leftMotor.setPower(.5);
            rightMotor.setPower(.5);
            leftMotor2.setPower(.5);
            rightMotor2.setPower(.5);
            sleep(250);   
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            leftMotor2.setPower(0);
            rightMotor2.setPower(0);
                    
                    break;
                default://Not visible
                    
            }
            

        }
        telemetry.update();
        sleep(20000);
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
}
