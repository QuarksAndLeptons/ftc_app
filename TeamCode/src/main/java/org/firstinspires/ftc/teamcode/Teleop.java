package org.firstinspires.ftc.teamcode;

//Import FTC modules

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

//Define as teleop
@TeleOp(name = "Current Teleoperation", group = "Linear Opmode")

public class Teleop extends org.firstinspires.ftc.teamcode.Autonomous {

    //Declare runtime variable
    private ElapsedTime runtime = new ElapsedTime();

    //Instantiate hardware links
    Servo color_servo;
    Servo jewel_rotation_servo;
    public static final double JEWEL_ROTATION_AMOUNT = 0.005;

    //Define opmode
    @Override public void runOpMode() {
        //This code runs immediately after the "init" button is pressed.
        //Inform the user that the opmode has been initialized.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Initialize hardware variables by paring them to motors in
        //"hardwareMap"
        //Initialize servos
        color_servo = hardwareMap.get(Servo.class, "jewel_servo");
        jewel_rotation_servo = hardwareMap.get(Servo.class, "jewel_rotation_servo");
        telemetry.addData("Status", "Initializing motors");

        //Reverse the direction of the right motors so the robot drives
        //forward when all motors are set to 1.0
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor2.setDirection(DcMotor.Direction.REVERSE);

        //Inform the user that he needs to press the play button
        telemetry.addData("Status", "Waiting for play button");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Inform the user of the current time
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // Set Motors to 0
            leftDrive(0);
            rightDrive(0);
            //Control the chassis
            leftDrive((gamepad1.left_stick_y - gamepad1.right_stick_x)/2);
            leftDrive((gamepad1.left_stick_y - gamepad1.right_stick_x)/2);
            rightDrive((gamepad1.left_stick_y + gamepad1.right_stick_x)/2);
            rightDrive((gamepad1.left_stick_y + gamepad1.right_stick_x)/2);
            telemetry.addData("Left motor power", leftMotor.getPower());
            telemetry.addData("Right motor power", leftMotor.getPower());

            //Gamepad buttons
            if (gamepad1.a) {
                color_servo.setPosition(1.0);
            }

            if (gamepad1.b) {
                color_servo.setPosition(0.0);
            }

            if (gamepad1.x && jewel_rotation_servo.getPosition() > jewel_rotation_servo.MIN_POSITION+JEWEL_ROTATION_AMOUNT) {
                jewel_rotation_servo.setPosition(jewel_rotation_servo.getPosition()-JEWEL_ROTATION_AMOUNT);
            }

            if (gamepad1.y&&jewel_rotation_servo.getPosition()<jewel_rotation_servo.MAX_POSITION-JEWEL_ROTATION_AMOUNT) {
                jewel_rotation_servo.setPosition(jewel_rotation_servo.getPosition()+JEWEL_ROTATION_AMOUNT);
            }
            //add some debug data
            telemetry.addData("Buttons",(gamepad1.a?"A":"-")+(gamepad1.b?"B":"-")+(gamepad1.x?"X":"-")+(gamepad1.y?"Y":"-"));
            telemetry.addData("Dpad",(gamepad1.dpad_left?"L":"-")+(gamepad1.dpad_right?"R":"-")+(gamepad1.dpad_down?"D":"-")+(gamepad1.dpad_up?"U":"-"));
            telemetry.addData("Servo Position",jewel_rotation_servo.getPosition());
            //Update the telemetry
            telemetry.update();
        }
        telemetry.addData("Status", "Done");
        telemetry.update();


    }
}