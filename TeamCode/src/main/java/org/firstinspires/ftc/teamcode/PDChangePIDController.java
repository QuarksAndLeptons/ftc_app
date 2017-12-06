package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
public class PDChangePIDController extends LinearOpMode {

    // our DC motor.
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    DcMotorEx leftMotor2;
    DcMotorEx rightMotor2;


    public static final double NEW_P = 2.5;
    public static final double NEW_I = 0.0;
    public static final double NEW_D = 0.0;

    public void runOpMode() {
        // get reference to DC motor.
        leftMotor = (DcMotorEx)hardwareMap.dcMotor.get("leftFront");
        rightMotor = (DcMotorEx)hardwareMap.dcMotor.get("rightFront");
        leftMotor2 = (DcMotorEx)hardwareMap.dcMotor.get("leftRear");
        rightMotor2 = (DcMotorEx)hardwareMap.dcMotor.get("rightRear");

        // wait for start command.
        waitForStart();

        // get a reference to the motor controller and cast it as an extended functionality controller.
        // we assume it's a REV Robotics Expansion Hub (which supports the extended controller functions).
        DcMotorControllerEx motorControllerEx = (DcMotorControllerEx)leftMotor.getController();

        // get the port number of our configured motor.
        int motorIndexLeft = leftMotor.getPortNumber();
        int motorIndexRight = rightMotor.getPortNumber();
        int motorIndexLeft2 = leftMotor2.getPortNumber();
        int motorIndexRight2 = rightMotor2.getPortNumber();
        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDCoefficients pidOrigLeft = motorControllerEx.getPIDCoefficients(motorIndexLeft, DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients pidOrigRight = motorControllerEx.getPIDCoefficients(motorIndexRight, DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients pidOrigLeft2 = motorControllerEx.getPIDCoefficients(motorIndexLeft2, DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients pidOrigRight2 = motorControllerEx.getPIDCoefficients(motorIndexRight2, DcMotor.RunMode.RUN_USING_ENCODER);
        // change coefficients.
        PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        motorControllerEx.setPIDCoefficients(motorIndexLeft, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        // re-read coefficients and verify change.
        PIDCoefficients pidModified = motorControllerEx.getPIDCoefficients(motorIndexLeft, DcMotor.RunMode.RUN_USING_ENCODER);

        // display info to user.
        while(opModeIsActive()) {
            leftMotor.setVelocity(180, AngleUnit.DEGREES);
            leftMotor2.setVelocity(180, AngleUnit.DEGREES);
            rightMotor.setVelocity(180, AngleUnit.DEGREES);
            rightMotor2.setVelocity(180, AngleUnit.DEGREES);

            //Update telemetry for debugging purposes
            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.addData("P,I,D (pidOrigLeft)", "%.04f, %.04f, %.0f",
                    pidOrigLeft.p, pidOrigLeft.i, pidOrigLeft.d);
            telemetry.addData("P,I,D (pidOrigRight)", "%.04f, %.04f, %.0f",
                    pidOrigLeft.p, pidOrigRight.i, pidOrigRight.d);
            telemetry.addData("P,I,D (pidOrigLeft2)", "%.04f, %.04f, %.0f",
                    pidOrigLeft.p, pidOrigLeft2.i, pidOrigLeft2.d);
            telemetry.addData("P,I,D (pidOrigRight2)", "%.04f, %.04f, %.0f",
                    pidOrigLeft.p, pidOrigRight2.i, pidOrigRight2.d);
            telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
                    pidModified.p, pidModified.i, pidModified.d);
            telemetry.update();
        }
    }



}
