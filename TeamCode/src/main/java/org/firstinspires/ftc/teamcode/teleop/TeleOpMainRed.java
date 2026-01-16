package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.constants.AllianceColor;

/**
 * RED Alliance wrapper for TeleOpMain
 * This op mode provides the same functionality as TeleOpMain but is specifically
 * labeled for RED alliance use in the driver station.
 */
@TeleOp(name = "RED: TeleOp Main", group = "Linear OpMode")
public class TeleOpMainRed extends TeleOpMain {
    @Override
    public void runOpMode() {
        // Set alliance color to RED before running
        setAllianceColor(AllianceColor.RED);
        // Call TeleOpMain's runOpMode implementation
        super.runOpMode();
    }
}
