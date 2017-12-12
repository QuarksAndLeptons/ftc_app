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
            gyroDrive(.3, 28, 0);    // Drive FWD 30 inches
            gyroTurn(1, -45.0);                // Turn  CCW to -15 Degrees
            gyroHold(1., -45.0, 0.5); // Hold -5 Deg heading for a 1/2 second
            //gyroDrive(DRIVE_SPEED, 28, 0);    // Drive FWD 30 inches
            moveDropMotorTo(300, 0.6, 3.0); //Drop a block
            sleep(2000);
            // gyroDrive(DRIVE_SPEED, 3, 0);    // Drive FWD 29 inches
            gyroDrive(.5, -5, -90);    // Drive FWD 30 inchess

        } else {
            telemetry.addData("VuMark", "%s visible", vuMark);
            switch (vuMark.ordinal()) {
                case 1:       //left
                    gyroDrive(.3, 28, 0);    // Drive FWD 30 inches
                    gyroTurn(1, 45.0);                // Turn  CCW to -15 Degrees
                    gyroHold(1, 45.0, 0.5); // Hold -5 Deg heading for a 1/2 second
                    //gyroDrive(DRIVE_SPEED, 28, 0);    // Drive FWD 30 inches
                    moveDropMotorTo(300, 0.6, 3.0); //Drop a block
                    sleep(2000);
                   // gyroDrive(DRIVE_SPEED, 3, 0);    // Drive FWD 29 inches
                    gyroDrive(.3, -5, -90);    // Drive FWD 30 inches
                    break;

                case 2://Center
                    gyroDrive(.3, 34, 0);    // Drive FWD 30 inches
                    gyroTurn(1, 95.0);                // Turn  CCW to -15 Degrees
                    gyroHold(1, 95.0, 0.5); // Hold -5 Deg heading for a 1/2 second
                    //gyroDrive(DRIVE_SPEED, 28, 0);    // Drive FWD 30 inches
                    moveDropMotorTo(300, 0.6, 3.0); //Drop a block
                    sleep(2000);
                    // gyroDrive(DRIVE_SPEED, 3, 0);    // Drive FWD 29 inches
                    gyroDrive(.3, -5, -90);    // Drive FWD 30 inches
                    break;

                case 3://Right
                    gyroDrive(.3, 34, 0);    // Drive FWD 30 inches
                    gyroTurn(1, 95.0);                // Turn  CCW to -15 Degrees
                    gyroHold(1, 95.0, 0.5); // Hold -5 Deg heading for a 1/2 second
                    //gyroDrive(DRIVE_SPEED, 28, 0);    // Drive FWD 30 inches
                    moveDropMotorTo(300, 0.6, 3.0); //Drop a block
                    sleep(2000);
                    // gyroDrive(DRIVE_SPEED, 3, 0);    // Drive FWD 29 inches
                    gyroDrive(.3, -5, -90);    // Drive FWD 30 inches
                    break;


            }
            debugColorSensor(blueSensorColor);
            telemetry.addData("Status", "Done");
            telemetry.update();
            sleep(10000);

        }
    }
}





