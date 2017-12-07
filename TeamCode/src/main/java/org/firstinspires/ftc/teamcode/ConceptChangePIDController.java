

/**
 * Created by FRC-6419 on 12/5/2017.
 */

/**
 * Created by tom on 9/26/17.
 * This assumes that you are using a REV Robotics Expansion Hub
 * as your DC motor controller. This op mode uses the extended/enhanced
 * PID-related functions of the DcMotorControllerEx class.
 * The REV Robotics Expansion Hub supports the extended motor controller
 * functions, but other controllers (such as the Modern Robotics and
 * Hitechnic DC Motor Controllers) do not.
 */

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
 * as your DC motor controller.  This op mode uses the extended/enhanced
 * PID-related functions of the DcMotorEx class.  The REV Robotics Expansion Hub
 * supports the extended motor functions, but other controllers (such as the
 * Modern Robotics and Hitechnic DC Motor Controllers) do not.
 */




@Autonomous(name="Concept: Change PID Controller Lange", group = "Examples")
public class ConceptChangePIDController extends LinearOpMode {

    // our DC motor.
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor leftMotor2;
    DcMotor rightMotor2;


    public static final double NEW_P = 2.5;
    public static final double NEW_I = 0.0;
    public static final double NEW_D = 0.0;

    public void runOpMode() {
        // get reference to DC motor.
        leftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        rightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        leftMotor = hardwareMap.get(DcMotor.class, "leftRear");
        rightMotor = hardwareMap.get(DcMotor.class, "rightRear");

        // wait for start command.
        waitForStart();

        // get a reference to the motor controller and cast it as an extended functionality controller.
        // we assume it's a REV Robotics Expansion Hub (which supports the extended controller functions).
        DcMotorControllerEx motorControllerExL = (DcMotorControllerEx) leftMotor.getController();
        DcMotorControllerEx motorControllerExR = (DcMotorControllerEx) rightMotor.getController();


        // get the port number of our configured motor.
        int motorIndexL = ((DcMotorEx) leftMotor).getPortNumber();
        int motorIndexR = ((DcMotorEx) rightMotor).getPortNumber();

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDCoefficients pidOrigL = motorControllerExL.getPIDCoefficients(motorIndexL, DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients pidOrigR = motorControllerExR.getPIDCoefficients(motorIndexR, DcMotor.RunMode.RUN_USING_ENCODER);


        // change coefficients.
        PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        motorControllerExL.setPIDCoefficients(motorIndexL, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        motorControllerExR.setPIDCoefficients(motorIndexR, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);


        // re-read coefficients and verify change.
        PIDCoefficients pidModifiedL = motorControllerExL.getPIDCoefficients(motorIndexL, DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients pidModifiedR = motorControllerExR.getPIDCoefficients(motorIndexR, DcMotor.RunMode.RUN_USING_ENCODER);


        // display info to user.
        while (opModeIsActive()) {
            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.addData("P,I,D (origL)", "%.04f, %.04f, %.0f", pidOrigL.p, pidOrigL.i, pidOrigL.d);
            telemetry.addData("P,I,D (origR)", "%.04f, %.04f, %.0f", pidOrigR.p, pidOrigR.i, pidOrigR.d);
            telemetry.addData("P,I,D (modifiedL)", "%.04f, %.04f, %.04f", pidModifiedL.p, pidModifiedL.i, pidModifiedL.d);
            telemetry.addData("P,I,D (modifiedR)", "%.04f, %.04f, %.04f", pidModifiedR.p, pidModifiedR.i, pidModifiedR.d);
            telemetry.update();
        }
    }

   /* public void rightDrive() {
        rightMotor.setVelocity(180, AngleUnit.DEGREES);
        rightMotor2.setVelocity(180, AngleUnit.DEGREES);
    }

    public void leftDrive() {
        leftMotor.setVelocity(180, AngleUnit.DEGREES);
        leftMotor2.setVelocity(180, AngleUnit.DEGREES);
    }
    */
}