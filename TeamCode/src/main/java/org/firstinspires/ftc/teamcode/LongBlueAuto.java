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

        blueColorServo.setPosition(.9);
        jewelRotationServo.setPosition(.5);
        sleep(2000);


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
            telemetry.addData("VuMark", "not visible");
            grabLowerGlyphs(); //Grab initial glyph
            liftGlyphs(.8);
            sleep(1000);
            liftGlyphs(0);
            gyroTurn(TURN_SPEED, -15.0, 4);                // Turn 15 degrees to teh right
            gyroHold(TURN_SPEED, -15.0, 0.5); // Hold for half a second
            gyroDrive(DRIVE_SPEED, 29, -15, 10);    // Drive forward 29 inches
            gyroHold(TURN_SPEED, -15.0, 0.5); // Hold for half a second
            liftGlyphs(-.8);
            sleep(1000);
            releaseGlyphs(); //release initial glyph
            sleep(2000);
            gyroDrive(DRIVE_SPEED, -6, 180);    // Drive backward six inches

        } else {
            telemetry.addData("VuMark", "%s visible", vuMark);
            switch (vuMark.ordinal()) {
                case 1: //Left
                    gyroTurn(TURN_SPEED, -2.0, 2);                // Turn  CCW to -5 Degrees
                    gyroHold(TURN_SPEED, -2.0, 0.5); // Hold -5 Deg heading for a 1/2 second
                    gyroDrive(DRIVE_SPEED, 28, -5, 7);    // Drive FWD 30 inches
                    gyroHold(TURN_SPEED, -2.0, 0.5); // Hold -5 Deg heading for a 1/2 second
                    moveDropMotorTo(300, 0.6, 3.0); //Drop a block
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, 180);    // Drive FWD 30 inches
                    break;

                case 2://Center
                    gyroTurn(TURN_SPEED, -15.0, 3);                // Turn  CCW to -15 Degrees
                    gyroHold(TURN_SPEED, -15.0, 0.5); // Hold -15 Deg heading for a 1/2 second
                    gyroDrive(DRIVE_SPEED, 29, -15, 7);    // Drive FWD 29 inches
                    gyroHold(TURN_SPEED, -15.0, 0.5); // Hold -15 Deg heading for a 1/2 second
                    moveDropMotorTo(300, 0.6, 3.0); //Drop a block
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, 180);    // Drive FWD 29 inches
                    break;

                case 3://Right
                    gyroTurn(TURN_SPEED, -23.0, 4);                // Turn  CCW to -25 Degrees
                    gyroHold(TURN_SPEED, -23.0, 0.5); // Hold -25 Deg heading for a 1/2 second
                    gyroDrive(DRIVE_SPEED, 31, -23, 7);    // Drive FWD 29 inches
                    gyroHold(TURN_SPEED, -23.0, 0.5); // Hold -25 Deg heading for a 1/2 second
                    moveDropMotorTo(300, 0.6, 3.0); //Drop a block
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, 180);    // Drive FWD 29 inches
                    break;


            }
            debugColorSensor(blueSensorColor);
            telemetry.addData("Status", "Done");
            telemetry.update();
            sleep(10000);

        }
    }
}





