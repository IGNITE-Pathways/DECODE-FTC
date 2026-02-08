package org.firstinspires.ftc.teamcode.competition.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.RobotConstants.AllianceColor;
import org.firstinspires.ftc.teamcode.competition.base.OdometryTeleOp;

/**
 * Blue Alliance - Odometry-Based TeleOp
 *
 * Uses equations for RPM/hood adjustment based on odometry distance
 */
@TeleOp(name = "BLUE Odometry TeleOp", group = "Competition")
public class BlueOdometryTeleOp extends OdometryTeleOp {

    public BlueOdometryTeleOp() {
        this.alliance = AllianceColor.BLUE;
    }
}
