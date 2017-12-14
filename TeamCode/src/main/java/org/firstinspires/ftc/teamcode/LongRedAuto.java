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
        color_servo.setPosition(.1);
        rotation_servo.setPosition(.47);

        //Wait for start
        telemetry.addData("Status", "Waiting for play button");
        telemetry.update();
        waitForStart();

        //Start measuring gyro acceleration and activate Vuforia
        startAdvancedSensing();

        runtime.reset();


        debugColorSensor(redSensorColor);
        telemetry.update();

        color_servo.setPosition(1);
        rotation_servo.setPosition(.5);
        sleep(2000);


        debugColorSensor(redSensorColor);
        telemetry.update();

        if (redSensorColor.red() > redSensorColor.blue()) {  // is red // go froward knock red
            rotation_servo.setPosition(.2);
        }
        if (redSensorColor.red() < redSensorColor.blue()) { // not red // go back knock red
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
            gyroTurn(TURN_SPEED, 20.0);                // Turn  CCW to -15 Degrees
            gyroHold(TURN_SPEED, 20.0, 0.5); // Hold -15 Deg heading for a 1/2 second
            gyroDrive(DRIVE_SPEED, 29, 20);    // Drive FWD 29 inches
            gyroHold(TURN_SPEED, 20.0, 0.5); // Hold -15 Deg heading for a 1/2 second
            moveDropMotorTo(100, 0.6, 3.0); //Drop a block
            sleep(2000);
            //      gyroDrive(DRIVE_SPEED, 3, 0);    // Drive FWD 29 inches
            gyroDrive(DRIVE_SPEED, -6, 180);    // Drive FWD 29 inches



        } else {
            telemetry.addData("VuMark", "%s visible", vuMark);
            switch (vuMark.ordinal()) {
                case 1:
                    gyroTurn(TURN_SPEED, 28.0);                // Turn  CCW to -25 Degrees
                    gyroHold(TURN_SPEED, 28.0, 0.5); // Hold -25 Deg heading for a 1/2 second
                    gyroDrive(DRIVE_SPEED, 31, 28);    // Drive FWD 29 inches
                    gyroHold(TURN_SPEED, 28.0, 0.5); // Hold -25 Deg heading for a 1/2 second
                    moveDropMotorTo(100, 0.6, 3.0); //Drop a block
                    sleep(2000);
                    gyroDrive(DRIVE_SPEED, -6, 180);    // Drive FWD 29 inches

                    break;

                case 2://Center
                    gyroTurn(TURN_SPEED, 20.0);                // Turn  CCW to -15 Degrees
                    gyroHold(TURN_SPEED, 20.0, 0.5); // Hold -15 Deg heading for a 1/2 second
                    gyroDrive(DRIVE_SPEED, 29, 20);    // Drive FWD 29 inches
                    gyroHold(TURN_SPEED, 20.0, 0.5); // Hold -15 Deg heading for a 1/2 second
                    moveDropMotorTo(100, 0.6, 3.0); //Drop a block
                    sleep(2000);
              //      gyroDrive(DRIVE_SPEED, 3, 0);    // Drive FWD 29 inches
                    gyroDrive(DRIVE_SPEED, -6, 180);    // Drive FWD 29 inches
                    break;

                case 3://Right
                    gyroTurn(TURN_SPEED, 10.0);                // Turn  CCW to -5 Degrees
                    gyroHold(TURN_SPEED, 10.0, 0.5); // Hold -5 Deg heading for a 1/2 second
                    gyroDrive(DRIVE_SPEED, 29, 10.0);    // Drive FWD 30 inches
                    gyroHold(TURN_SPEED, 10.0, 0.5); // Hold -5 Deg heading for a 1/2 second
                    moveDropMotorTo(100, 0.6, 3.0); //Drop a block
                    sleep(2000);
                    // gyroDrive(DRIVE_SPEED, 3, 0);    // Drive FWD 29 inches
                    gyroDrive(DRIVE_SPEED, -6, 180);    // Drive FWD 30 inches
                    break;



            }
            debugColorSensor(blueSensorColor);
            telemetry.addData("Status", "Done");
            telemetry.update();
            sleep(10000);

        }
    }
}




