package org.firstinspires.ftc.teamcode;

//Import FTC modules

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

//Define as teleop
@TeleOp(name = "Current Teleoperation", group = "Linear Opmode")

public class Teleop extends Team6475Controls {

    //Define opmode
    @Override public void runOpMode() {
        //This code runs immediately after the "init" button is pressed.

        //Initialize the hardware
        initializeHardware();

        //Reset the motor modes so the robot doesn't drive erratically
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)\
        telemetry.addData("Status", "Waiting for play button");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Inform the user of the current time
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Trigger", gamepad2.left_trigger );
            //Control the chassis
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double angle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
            telemetry.addData("angle", angle);
            if (gamepad1.right_trigger < .5) {
                leftMotor.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);
                leftMotor2.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);
                rightMotor2.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
                rightMotor.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
            }
            if (gamepad1.right_trigger > .5) {
                leftMotor.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x)/2);
                leftMotor2.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x)/2);
                rightMotor2.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x)/2);
                rightMotor.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x)/2);
            }
            //Add telemetry data
            //    telemetry.addData("Lift motor power", liftMotor.getPower());
            telemetry.addData("Left motor power", leftMotor.getPower());
            telemetry.addData("Right motor power", rightMotor.getPower());

            if(gamepad1.a) {
                blueColorServo.setPosition(.1);
                jewelRotationServo.setPosition(.47);

            }

            //Lifting mechanism

            liftGlyphs(-gamepad2.left_stick_y);
            glyphGraber(-gamepad2.right_stick_y);

            //Drop Glyphs
            if(gamepad2.right_trigger>.5){
                grabLowerGlyphs();
            }
            if(gamepad2.right_bumper){
                grabUpperGlyphs();
            }
            if(gamepad2.left_trigger>.5){
                releaseUpperGlyphs();
            }
            if(gamepad2.left_bumper){
                releaseLowerGlyphs();
            }


            //add some debug data
            telemetry.addData("Buttons",(gamepad1.a?"A":"-")+(gamepad1.b?"B":"-")+(gamepad1.x?"X":"-")+(gamepad1.y?"Y":"-"));
            telemetry.addData("Dpad",(gamepad1.dpad_left?"L":"-")+(gamepad1.dpad_right?"R":"-")+(gamepad1.dpad_down?"D":"-")+(gamepad1.dpad_up?"U":"-"));
            telemetry.addData("Servo Position", jewelRotationServo.getPosition());
            //telemetry.addData("Lift Position", liftMotor.getCurrentPosition());
            //telemetry.addData("Lift Target", liftMotor.getTargetPosition());
            //telemetry.addData("Lift status", liftMotor.isBusy()?"I'm busy":"I got free time.");

            //Update the telemetry
            telemetry.update();
        }

        telemetry.addData("Status", "Done");
        telemetry.update();


    }
}