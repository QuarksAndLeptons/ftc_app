package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
/**
 * Tests the glyph-collection mechanism
 * Created by Micah on 12/19/17.
 */
@Autonomous(name = "Glyph Collection Test", group = "Concept")

public class Autonomous_Concept3 extends org.firstinspires.ftc.teamcode.Autonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Status", "Grabbing glyphs");
            telemetry.update();
            grabGlyphs();
            liftGlyphs(0.0);
            sleep(1000);

            telemetry.addData("Status", "Lifting glyph mechanism");
            telemetry.update();
            liftGlyphs(1.0);
            sleep(1000);

            telemetry.addData("Status", "Dropping glyphs");
            telemetry.update();
            liftGlyphs(0.0);
            releaseGlyphs();
            sleep(1000);

            telemetry.addData("Status", "Lowering glyph mechanism");
            telemetry.update();
            liftGlyphs(-0.8);
            sleep(1000);
        }
    }
}
