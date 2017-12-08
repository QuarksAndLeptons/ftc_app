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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "Long Blue Auto", group = "Linear Opmode")
// @Autonomous(...) is the other common choice

public class LongBlueAuto extends org.firstinspires.ftc.teamcode.Autonomous {


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Initalize the hardware
        initializeHardware();
/*

        //Set the initial servo positions
        color_servo.setPosition(.1);
        rotation_servo.setPosition(.47);

        //Wait for start
        telemetry.addData("Status", "Waiting for play button");
        telemetry.update();
        runtime.reset();
        waitForStart();


        boolean done = false;
        //The start button has been pressed.
        while (opModeIsActive() && done == false) {

            debugColorSensor(blueSensorColor);
            telemetry.update();

            color_servo.setPosition(1);
            rotation_servo.setPosition(.5);
            sleep(2000);


            debugColorSensor(blueSensorColor);
            telemetry.update();

            if (blueSensorColor.red() < blueSensorColor.blue()) {  // is red // go froward knock red
                rotation_servo.setPosition(.2);
            }
            if (blueSensorColor.red() > blueSensorColor.blue()) { // not red // go back knock red
                rotation_servo.setPosition(.8);
            }


            telemetry.update();

            sleep(2000);
            color_servo.setPosition(.35);
            rotation_servo.setPosition(.47);
            sleep(2000);

*/
            gyroDrive(DRIVE_SPEED, 48.0, 0.0);    // Drive FWD 48 inches
            gyroTurn( TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
            gyroHold( TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second
            gyroDrive(DRIVE_SPEED, 12.0, -45.0);  // Drive FWD 12 inches at 45 degrees
            gyroTurn( TURN_SPEED,  45.0);         // Turn  CW  to  45 Degrees
            gyroHold( TURN_SPEED,  45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
            gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
            gyroHold( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for a 1 second
            gyroDrive(DRIVE_SPEED,-48.0, 0.0);    // Drive REV 48 inches
            //gyroDrive(1,48,0);                               //drive forward
            //gyroTurn(1,-45);   //turn right cw is neg
            //sleep(2000);
            //dropMotor.setTargetPosition(400);                       //lift ramp to drop glyph
            //if(dropMotor.isBusy()) dropMotor.setPower(0.1);
            //sleep(2000);
            //gyroDrive(.3,-5,-45);

            //Finish code
            //sleep(10000);
            //done = true;
        }
       // debugColorSensor(blueSensorColor);
        //telemetry.addData("Status", "Done");
        //telemetry.update();
    }




