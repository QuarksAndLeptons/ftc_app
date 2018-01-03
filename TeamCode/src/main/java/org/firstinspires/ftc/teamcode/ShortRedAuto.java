

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


@Autonomous(name = "Short Red Auto", group = "Linear Opmode")

public class ShortRedAuto extends Team6475Controls {


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

        if (blueSensorColor.red() > blueSensorColor.blue()) {  // is red, go back knock blue
            jewelRotationServo.setPosition(.8);
        }
        if (blueSensorColor.red() < blueSensorColor.blue()) { // not red, go forward knock blue
            jewelRotationServo.setPosition(.2);
        }


        telemetry.update();

        sleep(1000);
        blueColorServo.setPosition(.35);
        jewelRotationServo.setPosition(.47);
        sleep(1500);


        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        telemetry.addData("Identified Vumark", vuMark.name());

        //Is VuMark unknown?
        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "not visible");    //Center
            gyroDrive(DRIVE_SPEED, 24, 0, 5);    // Drive forward 24 inches
            TurnToHeading(-51, .5);                     //Turn right 51 degrees
            gyroHold(TURN_SPEED, -51, 0.5); // Hold for a half-second
            gyroDrive(DRIVE_SPEED, 12, -51, 5);    // Drive forward 12 inches
            sleep(1000);
            releaseGlyphs(); //release initial glyph
            sleep(2000);
            gyroDrive(DRIVE_SPEED, -6, 129);    // Drive backward six inches
        }
        else { // This must be a VuMark
            telemetry.addData("VuMark", "%s visible", vuMark);
            switch (vuMark.ordinal()) {

                //TODO tuning is needed
                case 1: //Left
                    gyroDrive(DRIVE_SPEED, 24, 0, 5);    // Drive forward 24 inches
                    TurnToHeading(-40, .5);                     //Turn 40 degrees to the right
                    gyroHold(TURN_SPEED, -40.0, 0.5); // Hold for a half-second
                    gyroDrive(DRIVE_SPEED, 16, -40, 5);    // Drive forward 16 inches
                    sleep(1000);
                    releaseGlyphs(); //release initial glyph
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, 90);    // Drive backward six inches
                    break;

                case 2://Center
                    telemetry.addData("VuMark", "not visible");
                    gyroDrive(DRIVE_SPEED, 24, 0, 5);    // Drive forward 24 inches
                    TurnToHeading(-51, .5);                     //Turn right 51 degrees
                    gyroHold(TURN_SPEED, -51, 0.5); // Hold for a half-second
                    gyroDrive(DRIVE_SPEED, 12, -51, 5);    // Drive forward 12 inches
                    sleep(1000);
                    releaseGlyphs(); //release initial glyph
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, 129);    // Drive backward six inches
                    break;

                case 3://Right
                    telemetry.addData("VuMark", "not visible");
                    gyroDrive(DRIVE_SPEED, 44, 0, 10);    // Drive forward 44 inches
                    TurnToHeading(-135, .5);                    //Turn 135 degrees to the right
                    gyroHold(TURN_SPEED, -135, 0.5); // Hold for a half-second
                    gyroDrive(DRIVE_SPEED, 16, -135, 5);    // Drive forward 16 inches
                    sleep(1000);
                    releaseGlyphs(); //release initial glyph
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, 90);    // Drive backward six inches
                    break;

            }
        }
        debugColorSensor(blueSensorColor);
        telemetry.addData("Status", "Done");
        telemetry.update();
        sleep(10000);
    }
}







