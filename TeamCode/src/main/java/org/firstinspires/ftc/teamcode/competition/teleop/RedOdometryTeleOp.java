package org.firstinspires.ftc.teamcode.competition.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.RobotConstants.AllianceColor;
import org.firstinspires.ftc.teamcode.competition.base.OdometryTeleOp;

/**
 * Red Alliance - Odometry-Based TeleOp
 *
 * Uses equations for RPM/hood adjustment based on odometry distance
 */
@TeleOp(name = "RED Odometry TeleOp", group = "Competition")
public class RedOdometryTeleOp extends OdometryTeleOp {

    public RedOdometryTeleOp() {
        this.alliance = AllianceColor.RED;
    }
}
