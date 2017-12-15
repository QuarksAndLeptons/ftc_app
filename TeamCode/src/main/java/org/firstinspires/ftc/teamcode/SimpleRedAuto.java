/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "Red Jewell Auto", group = "Linear Opmode")
// @Autonomous(...) is the other common choice

public class SimpleRedAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor; // = null;
    DcMotor rightMotor; //= null;
    DcMotor leftMotor2; //= null;
    DcMotor rightMotor2; // = null;
    Servo color_servo;
    Servo glyph_servo;
    Servo rotation_servo;
    ColorSensor blueSensorColor;
    ColorSensor redSensorColor;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotor = hardwareMap.dcMotor.get("leftFront");
        rightMotor = hardwareMap.dcMotor.get("rightFront");
        leftMotor2 = hardwareMap.dcMotor.get("leftRear");
        rightMotor2 = hardwareMap.dcMotor.get("rightRear");
        color_servo = hardwareMap.get(Servo.class, "jewelServo");
        rotation_servo = hardwareMap.get(Servo.class, "jewelRotationServo");
        blueSensorColor = hardwareMap.get(ColorSensor.class, "BlueColorSensor");
        redSensorColor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "RedColorSensor");
        float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;
        double done = 0;
        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        telemetry.addData("Status", "Initializing motors");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // Wait for the game to start (driver presses PLAY)
        color_servo.setPosition(.1);
        rotation_servo.setPosition(.47);

        waitForStart();
        telemetry.addData("Status", "Waiting for play button");
        runtime.reset();

        while (opModeIsActive()) {
            Color.RGBToHSV((int) (redSensorColor.red() * SCALE_FACTOR),
                    (int) (redSensorColor.green() * SCALE_FACTOR),
                    (int) (redSensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
            telemetry.addData("Red  ", redSensorColor.red());
            telemetry.addData("Green", redSensorColor.green());
            telemetry.addData("Blue ", redSensorColor.blue());
            telemetry.update();



            color_servo.setPosition(1);
            rotation_servo.setPosition(.5);
            sleep(2000);

            Color.RGBToHSV((int) (redSensorColor.red() * SCALE_FACTOR),
                    (int) (redSensorColor.green() * SCALE_FACTOR),
                    (int) (redSensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
            telemetry.addData("Red  ", redSensorColor.red());
            telemetry.addData("Green", redSensorColor.green());
            telemetry.addData("Blue ", redSensorColor.blue());
            telemetry.update();

            if (redSensorColor.red() > redSensorColor.blue() && done == 0) {  // is red // go froward knock red
                rotation_servo.setPosition(.2);
                done = 1;
            }
            if (redSensorColor.red() < redSensorColor.blue() && done == 0) { // not red // go back knock red
                rotation_servo.setPosition(.8);
                done = 1;
            }

            Color.RGBToHSV((int) (redSensorColor.red() * SCALE_FACTOR),
                    (int) (redSensorColor.green() * SCALE_FACTOR),
                    (int) (redSensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
            telemetry.addData("Red  ", redSensorColor.red());
            telemetry.addData("Green", redSensorColor.green());
            telemetry.addData("Blue ", redSensorColor.blue());
            telemetry.addData("Done ", done);
            telemetry.update();

            sleep(2000);
            color_servo.setPosition(.35);
            rotation_servo.setPosition(.47);
            sleep(20000000);

        }

    }
}


