package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.constants.AllianceColor;

/**
 * BLUE Alliance wrapper for TeleOpMain
 * This op mode provides the same functionality as TeleOpMain but is specifically
 * labeled for BLUE alliance use in the driver station.
 */
@TeleOp(name = "BLUE: TeleOp Main", group = "Linear OpMode")
public class TeleOpMainBlue extends TeleOpMain {
    @Override
    public void runOpMode() {
        // Set alliance color to BLUE before running
        setAllianceColor(AllianceColor.BLUE);
        // Call TeleOpMain's runOpMode implementation
        super.runOpMode();
    }
}
