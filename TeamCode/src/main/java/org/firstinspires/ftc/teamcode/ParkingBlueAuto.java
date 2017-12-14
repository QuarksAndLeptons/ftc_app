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


@Autonomous(name = "Blue Parking Auto", group = "Linear Opmode")
// @Autonomous(...) is the other common choice

public class ParkingBlueAuto extends org.firstinspires.ftc.teamcode.Autonomous {


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Initalize the hardware
        initializeHardware();


        //Set the initial servo positions
        blueColorServo.setPosition(.1);
        jewelRotationServo.setPosition(.47);

        //Wait for start
        telemetry.addData("Status", "Waiting for play button");
        runtime.reset();
        waitForStart();


        boolean done = false;
        //The start button has been pressed.
        while (opModeIsActive() && done == false) {

            debugColorSensor(blueSensorColor);
            telemetry.update();

            blueColorServo.setPosition(1);
            jewelRotationServo.setPosition(.5);
            sleep(2000);


            debugColorSensor(blueSensorColor);
            telemetry.update();

            if (blueSensorColor.red() < blueSensorColor.blue()) {  // is red // go froward knock red
                jewelRotationServo.setPosition(.2);
            }
            if (blueSensorColor.red() > blueSensorColor.blue()) { // not red // go back knock red
                jewelRotationServo.setPosition(.8);
            }


            telemetry.update();

            sleep(2000);
            blueColorServo.setPosition(.35);
            jewelRotationServo.setPosition(.47);

            driveForwardDistance(32);


            //Finish code
            sleep(10000);
            done = true;
        }
        debugColorSensor(blueSensorColor);
        telemetry.addData("Status", "Done");
        telemetry.update();
    }

}


