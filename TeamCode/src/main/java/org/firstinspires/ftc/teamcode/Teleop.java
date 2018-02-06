package org.firstinspires.ftc.teamcode;

//Import FTC modules

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

//Define as teleop
@TeleOp(name = "Competition Teleop", group = "Linear Opmode")

public class Teleop extends Team6475Controls {

    boolean singleDriverControlsEnabled = false; //Allows single-driver controls, hopefully always false

    @Override
    public void runOpMode() {
        // Initialize the hardware
        initializeHardware();

        // Reset the motor modes so the robot doesn't drive erratically
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize and retract servos
        blueColorServo.setPosition(.1);
        jewelRotationServo.setPosition(.47);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Waiting for start boi.");
        telemetry.update();
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Retrieve sensor data from the REV hub's built-in accelerometer
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double angle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));

            // Control the chassis
            // Default drive speed is half power
            if (gamepad1.right_trigger < .5) {
                leftMotor.setPower((-gamepad1.left_stick_y + gamepad1.right_stick_x) / 2.0);
                leftMotor2.setPower((-gamepad1.left_stick_y + gamepad1.right_stick_x) / 2.0);
                rightMotor2.setPower((-gamepad1.left_stick_y - gamepad1.right_stick_x) / 2.0);
                rightMotor.setPower((-gamepad1.left_stick_y - gamepad1.right_stick_x) / 2.0);
            }
            // Enable full power by holding right trigger
            if (gamepad1.right_trigger > .5) {
                leftMotor.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
                leftMotor2.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
                rightMotor2.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_x);
                rightMotor.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_x);
            }

            // Enable quarter power by holding left trigger
            if (gamepad1.left_trigger > .5) {
                leftMotor.setPower((-gamepad1.left_stick_y + gamepad1.right_stick_x) / 4.0);
                leftMotor2.setPower((-gamepad1.left_stick_y + gamepad1.right_stick_x) / 4.0);
                rightMotor2.setPower((-gamepad1.left_stick_y - gamepad1.right_stick_x) / 4.0);
                rightMotor.setPower((-gamepad1.left_stick_y - gamepad1.right_stick_x) / 4.0);
            }

            // Switch driver modes when the first controller's left stick button is pressed
            if(gamepad1.left_stick_button) {
                singleDriverControlsEnabled = !singleDriverControlsEnabled;
                // Lock the program until the user has stopped pressing the button to
                // prevent driver mode from oscillating
                while(gamepad1.left_stick_button&&opModeIsActive());
            }

            // SINGLE DRIVER CONTROLS
            if(singleDriverControlsEnabled){
                // Glyph-related code
                if (gamepad1.dpad_up) liftGlyphs(1);
                if (gamepad1.dpad_down) liftGlyphs(-1);
                else liftGlyphs(0);
                if (gamepad1.right_trigger > .5) grabGlyphs();
                if (gamepad1.left_trigger > .5) releaseGlyphs();

                //End game controls
                if (-gamepad1.right_stick_y > 0) {
                    relicExtendMotor.setPower(gamepad1.right_stick_y);
                    relicRetractMotor.setPower(-.1);
                }
                if (-gamepad1.right_stick_y < 0) {
                    relicExtendMotor.setPower(.1);
                    relicRetractMotor.setPower(gamepad1.right_stick_y);
                }
                if (-gamepad1.right_stick_y == 0) {
                    relicExtendMotor.setPower(0);
                    relicRetractMotor.setPower(0);
                }
                if (gamepad1.a) grabRelic();
                if (gamepad1.x) releaseRelic();
                if (gamepad1.y) liftRelic();
                if (gamepad1.b) dropRelic();
            }// end of single-driver controls
            // TWO-DRIVER CONTROLS
            else {
                // Glyph-related controls
                if (gamepad2.dpad_up) liftGlyphs(1);
                if (gamepad2.dpad_down) liftGlyphs(-1);
                else liftGlyphs(0);
                if (gamepad2.right_trigger > .5) grabGlyphs();
                if (gamepad2.left_trigger > .5) releaseGlyphs();

                // End-game controls
                if (-gamepad2.right_stick_y > 0) {
                    relicExtendMotor.setPower(gamepad2.right_stick_y);
                    relicRetractMotor.setPower(-.1);
                }
                if (-gamepad2.right_stick_y < 0) {
                    relicExtendMotor.setPower(.1);
                    relicRetractMotor.setPower(gamepad2.right_stick_y);
                }
                if (-gamepad2.right_stick_y == 0) {
                    relicExtendMotor.setPower(0);
                    relicRetractMotor.setPower(0);
                }
                if (gamepad2.x) grabRelic();
                if (gamepad2.b) releaseRelic();
                if (gamepad2.y) liftRelic();
                if (gamepad2.a) dropRelic();
            }// end of two-driver controls

            // Update the telemetry
            // Inform the user of important debug information
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Trigger", gamepad2.left_trigger);
            telemetry.addData("angle", angle);
            telemetry.addData("Single-driver controls enabled", singleDriverControlsEnabled);
            // Add telemetry data related to motors
            telemetry.addData("Left motor power", leftMotor.getPower());
            telemetry.addData("Right motor power", rightMotor.getPower());
            telemetry.addData("Relic motor power", relicExtendMotor.getPower());
            // Update telemetry
            telemetry.update();
        }// end of main teleop loop

        telemetry.addData("Status", "Done");
        telemetry.update();
    }// end of opMode()
}// end of class
