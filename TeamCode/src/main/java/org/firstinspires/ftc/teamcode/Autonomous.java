package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by snowflake6419 on 11/7/17.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Autonomous")
public class Autonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Info", "Wait for program start");
        telemetry.update();
        waitForStart();
        telemetry.addData("Info", "Program began");
        telemetry.update();
        sleep(1000);
        telemetry.addData("Info", "Program done!");
        telemetry.update();
    }
}
