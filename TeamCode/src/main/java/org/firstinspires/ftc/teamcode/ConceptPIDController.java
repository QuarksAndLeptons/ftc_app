package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "Concept Pid Controller", group = "Linear Opmode")
// @Team6475Controls(...) is the other common choice

public class ConceptPIDController extends Team6475Controls {


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Initalize the hardware
        initializeHardware();


        //Wait for start
        telemetry.addData("Status", "Waiting for play button");
        telemetry.update();
        waitForStart();

        //Start measuring gyro acceleration and activate Vuforia
        startAdvancedSensing();

        runtime.reset();

        Drive(.5,24,10,10);

        //TurnToHeading(-90,.5);


        telemetry.update();


    }
}