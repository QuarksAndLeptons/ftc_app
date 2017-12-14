package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

/**
 * Created by tom on 9/26/17.
 * This assumes that you are using a REV Robotics Expansion Hub
 * as your DC motor controller. This op mode uses the extended/enhanced
 * PID-related functions of the DcMotorControllerEx class.
 * The REV Robotics Expansion Hub supports the extended motor controller
 * functions, but other controllers (such as the Modern Robotics and
 * Hitechnic DC Motor Controllers) do not.
 */

@Autonomous(name="Concept: Change PID Controller", group = "Examples")
public class Autonomous_Concept extends LinearOpMode {

    // our DC motor.
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor leftMotor2;
    DcMotor rightMotor2;


    public static final double NEW_P = 2.5;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;

    public void runOpMode() {
        // get reference to DC motor.
        leftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        rightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        leftMotor2 = hardwareMap.get(DcMotor.class, "leftRear");
        rightMotor2 = hardwareMap.get(DcMotor.class, "rightRear");

        // wait for start command.
        waitForStart();

        // get a reference to the motor controller and cast it as an extended functionality controller.
        // we assume it's a REV Robotics Expansion Hub (which supports the extended controller functions).
        DcMotorControllerEx lmotorControllerEx = (DcMotorControllerEx) leftMotor.getController();
        DcMotorControllerEx rmotorControllerEx = (DcMotorControllerEx) rightMotor.getController();

        // get the port number of our configured motor.
        int lmotorIndex = ((DcMotorEx) leftMotor).getPortNumber();
        int rmotorIndex = ((DcMotorEx) rightMotor).getPortNumber();

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDCoefficients lpidOrig = lmotorControllerEx.getPIDCoefficients(lmotorIndex, DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients rpidOrig = rmotorControllerEx.getPIDCoefficients(rmotorIndex, DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients.
        PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        lmotorControllerEx.setPIDCoefficients(lmotorIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        rmotorControllerEx.setPIDCoefficients(rmotorIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        // re-read coefficients and verify change.
        PIDCoefficients lpidModified = lmotorControllerEx.getPIDCoefficients(lmotorIndex, DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients rpidModified = rmotorControllerEx.getPIDCoefficients(rmotorIndex, DcMotor.RunMode.RUN_USING_ENCODER);

        // display info to user.
        while(opModeIsActive()) {
            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f",
                    lpidOrig.p, lpidOrig.i, lpidOrig.d);
            telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
                    lpidModified.p, lpidModified.i, lpidModified.d);
            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f",
                    rpidOrig.p, rpidOrig.i, rpidOrig.d);
            telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
                    rpidModified.p, rpidModified.i, rpidModified.d);
            telemetry.update();

        }

    }
}

