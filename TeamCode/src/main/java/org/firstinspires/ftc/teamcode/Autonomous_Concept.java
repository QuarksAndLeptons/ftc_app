package org.firstinspires.ftc.teamcode;
//Import standard FTC libraries

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

//Import hardware libraries
//Import Gyro
//Import special FTC-related libraries
//Import navigation libraries


//https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotorControllerEx.html

@Autonomous(name = "Concept Autonomous", group = "Linear Opmode")
public class Autonomous_Concept extends org.firstinspires.ftc.teamcode.Autonomous {

    /**
     * Created by tom on 9/26/17.
     * This assumes that you are using a REV Robotics Expansion Hub
     * as your DC motor controller. This op mode uses the extended/enhanced
     * PID-related functions of the DcMotorControllerEx class.
     * The REV Robotics Expansion Hub supports the extended motor controller
     * functions, but other controllers (such as the Modern Robotics and
     * Hitechnic DC Motor Controllers) do not.
     */



        public static final double NEW_P = 2.5;
        public static final double NEW_I = 0.1;
        public static final double NEW_D = 0.2;

        public void runOpMode() {

            // wait for start command.
            waitForStart();

            leftDrive(.5);
            rightDrive(.5);

            // get a reference to the motor controller and cast it as an extended functionality controller.
            // we assume it's a REV Robotics Expansion Hub (which supports the extended controller functions).
            DcMotorControllerEx leftMotorControllerEx = (DcMotorControllerEx)leftMotor.getController();
            DcMotorControllerEx rightMotorControllerEx = (DcMotorControllerEx)rightMotor.getController();


            // get the port number of our configured motor.
            int leftMotorIndex = ((DcMotorEx)leftMotor).getPortNumber();
            int rightMotorIndex = ((DcMotorEx)rightMotor).getPortNumber();

            // get the PID coefficients for the RUN_USING_ENCODER  modes.
            PIDCoefficients lPidOrig = leftMotorControllerEx.getPIDCoefficients(leftMotorIndex, DcMotor.RunMode.RUN_USING_ENCODER);
            PIDCoefficients rPidOrig = rightMotorControllerEx.getPIDCoefficients(rightMotorIndex, DcMotor.RunMode.RUN_USING_ENCODER);

            // change coefficients.
            PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
            leftMotorControllerEx.setPIDCoefficients(leftMotorIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
            rightMotorControllerEx.setPIDCoefficients(rightMotorIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);


            // re-read coefficients and verify change.
            PIDCoefficients lPidModified = leftMotorControllerEx.getPIDCoefficients(leftMotorIndex, DcMotor.RunMode.RUN_USING_ENCODER);
            PIDCoefficients rPidModified = rightMotorControllerEx.getPIDCoefficients(rightMotorIndex, DcMotor.RunMode.RUN_USING_ENCODER);

            // display info to user.
            while(opModeIsActive()) {
                telemetry.addData("Runtime", "%.03f", getRuntime());
                telemetry.addData("Left P,I,D (orig)", "%.04f, %.04f, %.0f",
                        lPidOrig.p, lPidOrig.i, lPidOrig.d);
                telemetry.addData("Left P,I,D (modified)", "%.04f, %.04f, %.04f",
                        lPidModified.p, lPidModified.i, lPidModified.d);
                telemetry.addData("Right P,I,D (orig)", "%.04f, %.04f, %.0f",
                        rPidOrig.p, rPidOrig.i, rPidOrig.d);
                telemetry.addData("Left P,I,D (modified)", "%.04f, %.04f, %.04f",
                        rPidModified.p, rPidModified.i, rPidModified.d);
                telemetry.update();
            }
        }
    }







