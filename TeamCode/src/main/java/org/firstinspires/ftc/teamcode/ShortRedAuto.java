

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


@Autonomous(name = "Short Red Auto", group = "Linear Opmode")

public class ShortRedAuto extends org.firstinspires.ftc.teamcode.Autonomous {


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
//TODO tune new angles and distances for new drop mechanism
            telemetry.addData("VuMark", "not visible");
            gyroDrive(DRIVE_SPEED, 46, -1, 5);    // Drive forward 46 inches
            gyroTurn(TURN_SPEED, -60.0, 5);               // Turn left 60 Degrees
            gyroHold(TURN_SPEED, -60.0, 0.5); // Stop for 1/2 second, holding direction
            gyroDrive(DRIVE_SPEED, 2, -95, 5);    // Drive forward 2 inches while turning 35 degrees
            sleep(1000);
            releaseGlyphs(); //release initial glyph
            sleep(2000);
            gyroDrive(DRIVE_SPEED, -6, 90);    // Drive backward six inches
        }
        else { // This must be a VuMark
            telemetry.addData("VuMark", "%s visible", vuMark);
            switch (vuMark.ordinal()) {
                case 1: //Left
                    gyroDrive(DRIVE_SPEED, 30, -1, 5);    // Drive forward 30 inches
                    gyroTurn(TURN_SPEED, -95.0, 5);                // Turn 65 Degrees to the right
                    gyroHold(TURN_SPEED, -95.0, 0.5); // Hold for a half-second
                    gyroDrive(DRIVE_SPEED, 3, -95, 5);    // Drive forward 3 inches
                    sleep(1000);
                    releaseGlyphs(); //release initial glyph
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, 90);    // Drive backward six inches
                    break;

                case 2://Center
                    telemetry.addData("VuMark", "not visible");
                    gyroDrive(DRIVE_SPEED, 38, -1);    // Drive forward 38 inches
                    gyroTurn(TURN_SPEED, -100.0);                // Turn 100 degrees to the left
                    gyroHold(TURN_SPEED, -100.0, 0.5); // Hold for a half-second
                    gyroDrive(DRIVE_SPEED, 2, -95);    // Drive forward 2 inches
                    sleep(1000);
                    releaseGlyphs(); //release initial glyph
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, 90);    // Drive backward six inches
                    break;

                case 3://Right
                    telemetry.addData("VuMark", "not visible");
                    gyroDrive(DRIVE_SPEED, 36, 1, 5);    // Drive forward 36 inches
                    gyroTurn(TURN_SPEED, -60.0, 5);                // Turn 60 degrees to the left
                    gyroHold(TURN_SPEED, -60.0, 0.5); // Hold for a half-second
                    gyroDrive(DRIVE_SPEED, 5, -65, 3);    // Drive forward 5 inches
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







