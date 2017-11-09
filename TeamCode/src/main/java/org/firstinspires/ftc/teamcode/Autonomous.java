package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by snowflake6419 on 11/7/17.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Autonomous")
public class Autonomous extends LinearOpMode {
    /*
    These motors drive the chassis.
    WARNING:  leftMotor1 and leftMotor2 are mechanically paired, as with rightMotor1 and rightMotor2,
    so failure to reverse one paired motor, and not the other, will cause both motors to stall.
     */
    //Instantiate the chassis motors
    DcMotor leftMotor1,rightMotor1,leftMotor2,rightMotor2;

    //Instantiate the servos
    Servo color_servo, glyph_servo;

    //Instantiate the sensors
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;


    @Override
    public void runOpMode() throws InterruptedException {
        //The "init" button has been pressed.

        //Initialize the hardware devices
        leftMotor1 = hardwareMap.dcMotor.get("left_drive");
        rightMotor1 = hardwareMap.dcMotor.get("right_drive");
        leftMotor2 = hardwareMap.dcMotor.get("left_drive2");
        rightMotor2 = hardwareMap.dcMotor.get("right_drive2");
        color_servo = hardwareMap.get(Servo.class, "jewel_servo");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        //End of hardware initialization

        //Miscellaneous commands
        telemetry.addData("Info", "Wait for program start");
        telemetry.update();

        //We're done initializing.  Waiting for the start button
        waitForStart();

        //The start button has been pressed
        telemetry.addData("Info", "Program began");
        telemetry.update();
        sleep(1000);
        telemetry.addData("Info", "Program done!");
        telemetry.update();
    }
}
