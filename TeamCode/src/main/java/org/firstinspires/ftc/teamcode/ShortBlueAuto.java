package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


@Autonomous(name = "Short Blue Auto", group = "Linear Opmode")
// @Team6475Controls(...) is the other common choice

public class ShortBlueAuto extends Team6475Controls {


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
        blueColorServo.setPosition(.95);
        grabGlyphs();
        liftGlyphs(.8);
        sleep(2000);

        blueColorServo.setPosition(.93);
        jewelRotationServo.setPosition(.5);
        sleep(1500);


        debugColorSensor(blueSensorColor);
        telemetry.update();

        if (blueSensorColor.red() < blueSensorColor.blue()) {  // is red // go froward knock red
            jewelRotationServo.setPosition(.2);
        }
        if (blueSensorColor.red() > blueSensorColor.blue()) { // not red // go back knock red
            jewelRotationServo.setPosition(.8);
        }


        telemetry.update();

        sleep(1000);
        blueColorServo.setPosition(.36);
        jewelRotationServo.setPosition(.47);
        sleep(1000);


        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        telemetry.addData("Identified Vumark", vuMark.name());


        //Is VuMark unknown?
        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "not visible");    //Center
            gyroDrive(DRIVE_SPEED, 24, 0, 5);    // drive forward 24 inches
            turnToHeading(51, .5);                     //Turn left 51 degrees
            gyroHold(TURN_SPEED, 51, 0.5); // Hold for a half-second
            gyroDrive(DRIVE_SPEED, 12, 51, 5);    // drive forward 12 inches
            sleep(1000);
            releaseGlyphs(); //release initial glyph
            sleep(2000);
            gyroDrive(DRIVE_SPEED, -6, -129);    // drive backward six inches
        }
        else { // This must be a VuMark
            telemetry.addData("VuMark", "%s visible", vuMark);
            switch (vuMark.ordinal()) {
                case 1: //Left
                    gyroDrive(DRIVE_SPEED, 44, 0, 10);    // drive forward 44 inches
                    turnToHeading(135, .5);                    //Turn 135 degrees to the left
                    gyroHold(TURN_SPEED, 135, 0.5); // Hold for a half-second
                    gyroDrive(DRIVE_SPEED, 16, 135, 5);    // drive forward 16 inches
                    sleep(1000);
                    releaseGlyphs(); //release initial glyph
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, -45);    // drive backward six inches
                    break;

                case 2://Center
                    telemetry.addData("VuMark", "%s visible");
                    gyroDrive(DRIVE_SPEED, 24, 0, 5);    // drive forward 24 inches
                    turnToHeading(51, .5);                     //Turn left 51 degrees
                    gyroHold(TURN_SPEED, 51, 0.5); // Hold for a half-second
                    gyroDrive(DRIVE_SPEED, 12, 51, 5);    // drive forward 12 inches
                    sleep(1000);
                    releaseGlyphs(); //release initial glyph
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, -129);    // drive backward six inches
                    break;

                case 3://Right
                    telemetry.addData("VuMark", "not visible");
                    gyroDrive(DRIVE_SPEED, 24, 0, 5);    // drive forward 24 inches
                    turnToHeading(40, .5);                     //Turn 40 degrees to the right
                    //gyroTurn(TURN_SPEED, -95.0, 5);                // Turn 65 Degrees to the right
                    gyroHold(TURN_SPEED, 40.0, 0.5); // Hold for a half-second
                    gyroDrive(DRIVE_SPEED, 16, 40, 5);    // drive forward 16 inches
                    sleep(1000);
                    releaseGlyphs(); //release initial glyph
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, -90);    // drive backward six inches
                    break;

            }
        }
        debugColorSensor(blueSensorColor);
        telemetry.addData("Status", "Done");
        telemetry.update();
        sleep(10000);
    }
}







