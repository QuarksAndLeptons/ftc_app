package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


@Autonomous(name = "Short Blue Auto", group = "Linear Opmode")
// @Autonomous(...) is the other common choice

public class ShortBlueAuto extends org.firstinspires.ftc.teamcode.Autonomous {


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
        blueColorServo.setPosition(.35);
        jewelRotationServo.setPosition(.47);
        sleep(1000);


        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        telemetry.addData("Identified Vumark", vuMark.name());


        //Is VuMark unknown?
        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "not visible");
            gyroDrive(DRIVE_SPEED, 25, 1, 8);    // Drive forward 36 inches
            gyroTurn(TURN_SPEED, 53.0, 8);                // Turn left 60 degrees
            gyroHold(TURN_SPEED, 53.0, 0.5);             // Hold for half a second
            gyroDrive(DRIVE_SPEED, 7, 53, 8);     // Drive forward 5 inches
            sleep(1000);
            releaseGlyphs(); //release initial glyph
            sleep(2000);
            gyroDrive(DRIVE_SPEED, -6, -125);    // Drive backward six inches
        }
        else { // This must be a VuMark
            telemetry.addData("VuMark", "%s visible", vuMark);
            switch (vuMark.ordinal()) {
                case 1: //Left
                    gyroDrive(DRIVE_SPEED, 36, 1, 7);    // Drive forward 30 inches
                    gyroTurn(TURN_SPEED, 123, 5);                // Turn left 95 degrees
                    gyroHold(TURN_SPEED, 123, 0.5); // Hold for half a second
                    gyroDrive(DRIVE_SPEED, 6, 123, 5);    // Drive forward three inches
                    sleep(1000);
                    releaseGlyphs(); //release initial glyph
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, -60);    // Drive backward six inches
                    break;

                case 2://Center
                    telemetry.addData("VuMark", "%s visible");
                    gyroDrive(DRIVE_SPEED, 25, 1, 8);    // Drive forward 36 inches
                    gyroTurn(TURN_SPEED, 53.0, 8);                // Turn left 60 degrees
                    gyroHold(TURN_SPEED, 53.0, 0.5);             // Hold for half a second
                    gyroDrive(DRIVE_SPEED, 7, 53, 8);     // Drive forward 5 inches
                    sleep(1000);
                    releaseGlyphs(); //release initial glyph
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, -125);    // Drive backward six inches
                    break;

                case 3://Right
                    telemetry.addData("VuMark", "not visible");
                    gyroDrive(DRIVE_SPEED, 26, 1, 8);    // Drive forward 36 inches
                    gyroTurn(TURN_SPEED, 45.0, 8);                // Turn left 60 degrees
                    gyroHold(TURN_SPEED, 45.0, 0.5);             // Hold for half a second
                    gyroDrive(DRIVE_SPEED, 8, 45, 8);     // Drive forward 5 inches
                    sleep(1000);
                    releaseGlyphs(); //release initial glyph
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, -135);    // Drive backward six inchess
                    break;

            }
        }
        debugColorSensor(blueSensorColor);
        telemetry.addData("Status", "Done");
        telemetry.update();
        sleep(10000);
    }
}







