package org.firstinspires.ftc.teamcode;
//Import standard FTC libraries

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

//Import hardware libraries
//Import Gyro
//Import special FTC-related libraries
//Import navigation libraries


@Autonomous(name = "Red_Autonomous", group = "Linear Opmode")
public class Red_Autonomous extends org.firstinspires.ftc.teamcode.Autonomous
{
    //Run this code when the "init" button is pressed
    @Override
    public void runOpMode() {
        //Initialize hardware
        initializeHardware();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        driveTime(2, -1);
    }

}







