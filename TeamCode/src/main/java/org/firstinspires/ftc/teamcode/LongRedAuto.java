/*
TODO Add a timer that says at 25 seconds drop block and back up
NOTE: a new autonomous method should (in theory) move the drop motor
to a certain position and give up in a certain amount of time.
Called moveDropMotorTo(...), this method is used if Vuforia is not recognized
or it refers to the center position.  It has not been tested at the time
of this commit
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


@Autonomous(name = "Long Red Auto", group = "Linear Opmode")
// @Autonomous(...) is the other common choice

public class LongRedAuto extends org.firstinspires.ftc.teamcode.Autonomous {


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
        telemetry.update();
        waitForStart();

        //Start measuring gyro acceleration and activate Vuforia
        startAdvancedSensing();

        runtime.reset();


        debugColorSensor(blueSensorColor);
        telemetry.update();

        //Grab initial glyph
        jewelRotationServo.setPosition(.5);
        blueColorServo.setPosition(.91);
        grabLowerGlyphs();
        liftGlyphs(.8);
        sleep(2000);
        glyphLifter.setPosition(.5);


        debugColorSensor(blueSensorColor);
        telemetry.update();

        if (blueSensorColor.red() > blueSensorColor.blue()) { // is red, go back knock blue
            jewelRotationServo.setPosition(.8);
        }
        if (blueSensorColor.red() < blueSensorColor.blue()) { // not red, go forward knock blue
            jewelRotationServo.setPosition(.2);
        }


        telemetry.update();

        sleep(1000);
        blueColorServo.setPosition(.35);
        jewelRotationServo.setPosition(.47);
        sleep(1000);


        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        telemetry.addData("Identified Vumark", vuMark.name());

//TODO tune new angles and distances for new drop mechanism

        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "not visible");
            gyroTurn(TURN_SPEED, 20.0,3);                // Turn  CCW to -15 Degrees
            gyroHold(TURN_SPEED, 20.0, 0.5); // Hold -15 Deg heading for a 1/2 second
            gyroDrive(DRIVE_SPEED, 29, 20, 9);    // Drive FWD 29 inches
            gyroHold(TURN_SPEED, 20.0, 0.5); // Hold -15 Deg heading for a 1/2 second
            sleep(1000);
            releaseGlyphs(); //release initial glyph
            sleep(2000);
            gyroDrive(DRIVE_SPEED, -6, 180);    // Drive backward six inches



        } else {
            telemetry.addData("VuMark", "%s visible", vuMark);
            switch (vuMark.ordinal()) {
                case 1:
                    gyroTurn(TURN_SPEED, 2.0, 1);                // Turn  CCW to -25 Degrees
                    gyroHold(TURN_SPEED, 27.0, 0.5); // Hold -25 Deg heading for a 1/2 second
                    gyroDrive(DRIVE_SPEED, 32, 28, 9);    // Drive FWD 29 inches
                    gyroHold(TURN_SPEED, 27.0, 0.5); // Hold -25 Deg heading for a 1/2 second
                    sleep(1000);
                    releaseGlyphs(); //release initial glyph
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, 180);    // Drive backward six inches

                    break;

                case 2://Center
                    gyroTurn(TURN_SPEED, 20.0, 3);                   // Turn 20 degrees to the left
                    gyroHold(TURN_SPEED, 20.0, 0.5);                // Hold for half a second
                    gyroDrive(DRIVE_SPEED, 29, 20, 9);        // Drive forward 29 inches
                    gyroHold(TURN_SPEED, 20.0, 0.5);                 // Hold for half a second
                    sleep(1000);
                    releaseGlyphs(); //release initial glyph
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, 180);    // Drive backward six inches
                    break;

                case 3://Right
                    gyroTurn(TURN_SPEED, 10.0, 2);                  // Turn  CCW to -5 Degrees
                    gyroHold(TURN_SPEED, 10.0, 0.5);                // Hold for half a second
                    gyroDrive(DRIVE_SPEED, 29, 10.0, 9);    // Drive FWD 30 inches
                    gyroHold(TURN_SPEED, 10.0, 0.5);                // Hold for half a second
                    sleep(1000);
                    releaseGlyphs(); //release initial glyph
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, 180);    // Drive backward six inches
                    break;



            }
            debugColorSensor(blueSensorColor);
            telemetry.addData("Status", "Done");
            telemetry.update();
            sleep(10000);

        }
    }
}





