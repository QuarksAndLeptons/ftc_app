package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name = "Concept Pid Controller", group = "Linear Opmode")
// @Team6475Controls(...) is the other common choice
@Disabled
public class ConceptPIDController extends Team6475Controls {
//TODO determine why it turns slightly at the conclusion on steps


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

        Drive(.5,24,0,10);
        gyroHold(.2,0,.5);

        TurnToHeading(-90,.5);
        gyroHold(.2,-90,.5);

        Drive(.5,24,-90,10);
        gyroHold(.2,-90,.5);


        telemetry.update();


    }
}