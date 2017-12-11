//Add a  timer that says at 25 seconds drop block and back up

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


@Autonomous(name = "Long Blue Auto", group = "Linear Opmode")
// @Autonomous(...) is the other common choice

public class LongBlueAuto extends org.firstinspires.ftc.teamcode.Autonomous {


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Initalize the hardware
        initializeHardware();


        //Set the initial servo positions
        color_servo.setPosition(.1);
        rotation_servo.setPosition(.47);

        //Wait for start
        telemetry.addData("Status", "Waiting for play button");
        telemetry.update();
        waitForStart();

        //Start measuring gyro acceleration and activate Vuforia
        startAdvancedSensing();

        runtime.reset();



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


            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            telemetry.addData("Identified Vumark", vuMark.name());

            if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "not visible");
                gyroTurn(TURN_SPEED, -5.0);                // Turn  CCW to -5 Degrees
                gyroHold(TURN_SPEED, -5.0, 0.5); // Hold -5 Deg heading for a 1/2 second
                gyroDrive(DRIVE_SPEED, 28, -5);    // Drive FWD 30 inches
                gyroHold(TURN_SPEED, -5.0, 0.5); // Hold -5 Deg heading for a 1/2 second
                dropMotor.setTargetPosition(300);                      //lift ramp to drop glyph
                if (dropMotor.isBusy()) dropMotor.setPower(.3);
                sleep(2000);
                gyroDrive(DRIVE_SPEED, -28, 175);    // Drive FWD 30 inches





            } else {
                telemetry.addData("VuMark", "%s visible", vuMark);
                switch (vuMark.ordinal()) {
                    case 1:
                        gyroTurn(TURN_SPEED, -5.0);                // Turn  CCW to -5 Degrees
                        gyroHold(TURN_SPEED, -5.0, 0.5); // Hold -5 Deg heading for a 1/2 second
                        gyroDrive(DRIVE_SPEED, 28, -5);    // Drive FWD 30 inches
                        gyroHold(TURN_SPEED, -5.0, 0.5); // Hold -5 Deg heading for a 1/2 second
                        dropMotor.setTargetPosition(300);                      //lift ramp to drop glyph
                        if (dropMotor.isBusy()) dropMotor.setPower(.3);
                        sleep(2000);
                        gyroDrive(DRIVE_SPEED, -28, 180);    // Drive FWD 30 inches



                        break;

                    case 2://Center
                        gyroTurn(TURN_SPEED, -15.0);                // Turn  CCW to -15 Degrees
                        gyroHold(TURN_SPEED, -15.0, 0.5); // Hold -15 Deg heading for a 1/2 second
                        gyroDrive(DRIVE_SPEED, 29, -15);    // Drive FWD 29 inches
                        gyroHold(TURN_SPEED, -15.0, 0.5); // Hold -15 Deg heading for a 1/2 second
                        dropMotor.setTargetPosition(300);                      //lift ramp to drop glyph
                        if (dropMotor.isBusy()) dropMotor.setPower(0.6);
                        sleep(2000);
                        gyroDrive(DRIVE_SPEED, -29, 180);    // Drive FWD 29 inches




                        break;

                    case 3://Right
                        gyroTurn(TURN_SPEED, -30.0);                // Turn  CCW to -25 Degrees
                        gyroHold(TURN_SPEED, -30.0, 0.5); // Hold -25 Deg heading for a 1/2 second
                        gyroDrive(DRIVE_SPEED, 31, -30);    // Drive FWD 29 inches
                        gyroHold(TURN_SPEED, -30.0, 0.5); // Hold -25 Deg heading for a 1/2 second
                        dropMotor.setTargetPosition(300);               //lift ramp to drop glyph
                        if (dropMotor.isBusy()) dropMotor.setPower(.5);
                        sleep(2000);
                        gyroDrive(DRIVE_SPEED, -31, 180);    // Drive FWD 29 inches



                }
                debugColorSensor(blueSensorColor);
                telemetry.addData("Status", "Done");
                telemetry.update();
                sleep(10000);

            }

    }

}





