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
// @Team6475Controls(...) is the other common choice

public class LongRedAuto extends Team6475Controls {


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
        grabGlyphs();
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
            grabGlyphs();
            gyroDrive(DRIVE_SPEED, 24, 0, 10);    // drive forward 24 inches
            turnToHeading(50,.4);                      //Turn left 50 degrees
            gyroHold(TURN_SPEED, 50.0, 0.5);           // Hold for half a second
            gyroDrive(DRIVE_SPEED, 16, 50, 10);    // drive forward 16 inches
            gyroHold(TURN_SPEED, 50.0, 0.5);           // Hold for half a second
            sleep(1000);
            releaseGlyphs(); //release initial glyph
            sleep(2000);
            gyroDrive(DRIVE_SPEED, -6, 130);            // drive backward 6 inches



        } else {
            telemetry.addData("VuMark", "%s visible", vuMark);
            switch (vuMark.ordinal()) {
                case 1://left
                    grabGlyphs();
                    gyroDrive(DRIVE_SPEED, 24, 0, 10);    // drive forward 24 inches
                    turnToHeading(50,.4);                      //Turn left 50 degrees
                    gyroHold(TURN_SPEED, 50.0, 0.5);           // Hold for half a second
                    gyroDrive(DRIVE_SPEED, 16, 50, 10);    // drive forward 16 inches
                    gyroHold(TURN_SPEED, 50.0, 0.5);           // Hold for half a second
                    sleep(1000);
                    releaseGlyphs(); //release initial glyph
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, 130);            // drive backward 6 inches
                    break;

                case 2://Center
                    telemetry.addData("VuMark", "not visible");
                    grabGlyphs();
                    gyroDrive(DRIVE_SPEED, 24, 0, 10);    // drive forward 24 inches
                    turnToHeading(40,.4);                          //turn left 40 degrees
                    gyroHold(TURN_SPEED, 40.0, 0.5);           // Hold for half a second
                    gyroDrive(DRIVE_SPEED, 11, 40, 10);    // drive forward 11 inches
                    gyroHold(TURN_SPEED, 40.0, 0.5);           // Hold for half a second
                    sleep(1000);
                    releaseGlyphs(); //release initial glyph
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, 180);            // drive backward 6 inches
                    break;

                case 3://Right
                    grabGlyphs();
                    turnToHeading(-15,.5);                       // Turn 15 degrees to the right
                    gyroHold(TURN_SPEED, -15.0, 0.5);            // Hold for half a second
                    gyroDrive(DRIVE_SPEED, 29, -15, 10);    // drive forward 29 inches
                    turnToHeading(45,.5);                      // Turn to a heading of 45 degrees to the left
                    gyroHold(TURN_SPEED, 45, 0.5);             // Hold for half a second
                    gyroDrive(DRIVE_SPEED, 9, 45, 10);    // drive forward 9 inches
                    sleep(1000);
                    releaseGlyphs(); //release initial glyph
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, 180, 10);    // drive backward six inches
                    telemetry.addData("VuMark", "not visible");
                    break;



            }
            debugColorSensor(blueSensorColor);
            telemetry.addData("Status", "Done");
            telemetry.update();
            sleep(10000);

        }
    }
}





