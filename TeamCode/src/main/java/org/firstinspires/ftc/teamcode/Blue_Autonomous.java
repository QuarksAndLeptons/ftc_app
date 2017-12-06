/*
TODO Connect to the phone wirelessly
TODO Use Autonomous_Concept to fix driveForward()
TODO Use Autonomous_Conept to fix turnClockwise()
TODO Write code for all of the glyph patterns
TODO Fine-tune the program to work flawlessly
 */
package org.firstinspires.ftc.teamcode;
//Import standard FTC libraries

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//Import hardware libraries
//Import Gyro
//Import special FTC-related libraries
//Import navigation libraries


@Autonomous(name = "Blue_Autonomous", group = "Linear Opmode")
public class Blue_Autonomous extends org.firstinspires.ftc.teamcode.Autonomous {

    //Run this code when the "init" button is pressed
    @Override
    public void runOpMode() {
//Insert Simple Blue Auto


        /*
        //Detect the Vumark
        telemetry.addData("Status", "VuMarking");
        telemetry.update();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        sleep(2000);
        //If Vumark not visible
        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "not visible");
            telemetry.update();
            encoderDrive(-.5, -21, -21, 10);
            CarTurnDegreeDirection(45, "Right");
            telemetry.addData("Status", "Successful");
            telemetry.update();
            encoderDrive(-.5, -10, -10, 10);  //(power,left inches, right inches, timeout)
            dropMotor.setTargetPosition(400);
            dropMotor.setPower(-0.25);
        } else {
            telemetry.addData("VuMark", "%s visible", vuMark);
            telemetry.update();
            //Choose what to do based on the Vumark the camera sees
            switch (vuMark.ordinal()) {
                case 1://Left
                    telemetry.addData("VuMark", "Left Action");
                    encoderDrive(-.3, -21, -21, 10);
                    turnClockwise(45);
                    telemetry.addData("Status", "Successful");
                    telemetry.update();
                    encoderDrive(-.3, -10, -10, 10);  //(power,left inches, right inches, timeout)
                    dropMotor.setTargetPosition(400);
                    dropMotor.setPower(-0.25);
                    break;

                case 2://Center
                    telemetry.addData("VuMark", "Center Action");
                    encoderDrive(.3, 23, 23, 10);
                    turnClockwise(30);
                    dropMotor.setTargetPosition(400);
                    dropMotor.setPower(-0.25);
                    encoderDrive(.2, 2, 2, 10);
                    break;

                case 3://Right
                    telemetry.addData("VuMark", "Right Action");
                    //encoderDrive(-.3,-24,-24,10);
                    //FIXME
                    break;
            }

        }

        //Autonomous code is finished
        telemetry.addData("Status", "Done!");
        telemetry.update();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        sleep(1000);
        */
        //End of Autonomous
    }
}







