package org.firstinspires.ftc.teamcode.OpModes.IndividualTest;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Test: Limelight Individual Test", group="Main")
public class LimelightTest extends OpMode {
    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.start();
        }
    }

    @Override
    public void loop() {
        if (limelight == null || !limelight.isConnected()) {
            telemetry.addLine("Limelight not connected");
        } else {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("ta", result.getTa());
                telemetry.addData("BotPose", result.getBotpose().toString());
            } else {
                telemetry.addLine("No AprilTag detected");
            }
        }
        telemetry.update();
    }
}
