package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "Concept Integrated PID", group = "Linear Opmode")
// @Team6475Controls(...) is the other common choice

public class ConceptIntegratedPID extends Team6475Controls {

    public static final double NEW_P = 2.5;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;

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


        setPID(2.5,.1,.2);
    }

}







