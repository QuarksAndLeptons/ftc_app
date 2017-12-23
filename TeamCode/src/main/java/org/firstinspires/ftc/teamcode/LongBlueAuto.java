package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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

        if (blueSensorColor.red() < blueSensorColor.blue()) {  // If it's red, go froward knock red
            jewelRotationServo.setPosition(.2);
        }
        if (blueSensorColor.red() > blueSensorColor.blue()) { //If it's blue go back knock red
            jewelRotationServo.setPosition(.8);
        }


        telemetry.update();

        sleep(1000);
        blueColorServo.setPosition(.35);
        jewelRotationServo.setPosition(.47);
        sleep(1000);



        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        telemetry.addData("Identified Vumark", vuMark.name());

        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            grabLowerGlyphs();
            gyroDrive(DRIVE_SPEED, 24, 0, 5);    // Drive forward 29 inches
            gyroTurn(TURN_SPEED, -30.0, 5);                // Turn 15 degrees to teh right
            gyroHold(TURN_SPEED, -30.0, 0.5); // Hold for half a second
            gyroDrive(DRIVE_SPEED, 6, -30, 5);    // Drive forward 29 inches
            gyroHold(TURN_SPEED, -30.0, 0.5); // Hold for half a second
            sleep(1000);
            releaseGlyphs(); //release initial glyph
            sleep(2000);
            gyroDrive(DRIVE_SPEED, -6, 180);    // Drive backward six inches

        } else {
            telemetry.addData("VuMark", "%s visible", vuMark);
            switch (vuMark.ordinal()) {
                case 1: //Left
                    grabLowerGlyphs();
                    gyroTurn(TURN_SPEED, 15.0, 4);                // Turn 15 degrees to teh right
                    gyroHold(TURN_SPEED, 15.0, 0.5); // Hold for half a second
                    gyroDrive(DRIVE_SPEED, 29, 15, 10);    // Drive forward 29 inches
                    gyroTurn(TURN_SPEED, -45.0, 4);                // Turn 15 degrees to teh right
                    gyroHold(TURN_SPEED, -45, 0.5); // Hold for half a second
                    gyroDrive(DRIVE_SPEED, 7, -45, 10);    // Drive forward 29 inches
                    sleep(1000);
                    releaseGlyphs(); //release initial glyph
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, 180);    // Drive backward six inches
                    break;

                case 2://Center
                    grabLowerGlyphs();
                    gyroDrive(DRIVE_SPEED, 24, 0, 5);    // Drive forward 29 inches
                    gyroTurn(TURN_SPEED, -30.0, 4);                // Turn 15 degrees to teh right
                    gyroHold(TURN_SPEED, -30.0, 0.5); // Hold for half a second
                    gyroDrive(DRIVE_SPEED, 6, -30, 5);    // Drive forward 29 inches
                    gyroHold(TURN_SPEED, -30.0, 0.5); // Hold for half a second
                    sleep(1000);
                    releaseGlyphs(); //release initial glyph
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, 180);    // Drive backward six inches
                    break;

                case 3://Right
                    grabLowerGlyphs();
                    gyroDrive(DRIVE_SPEED, 24, 0, 5);    // Drive forward 29 inches
                    gyroTurn(TURN_SPEED, -53.0, 4);                // Turn 15 degrees to teh right
                    gyroHold(TURN_SPEED, -53.0, 0.5); // Hold for half a second
                    gyroDrive(DRIVE_SPEED, 15, -53, 5);    // Drive forward 29 inches
                    gyroHold(TURN_SPEED, -53.0, 0.5); // Hold for half a second
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





