package org.firstinspires.ftc.teamcode.competition.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.RobotConstants.AllianceColor;
import org.firstinspires.ftc.teamcode.competition.base.CompetitionTeleOpBase;

/**
 * BLUE ALLIANCE Competition TeleOp - Turret tracks AprilTag ID 20
 *
 * GP1: Drive (sticks), Y = flywheel+ramp toggle, triggers = intake/eject
 * GP2: A = turret toggle, manual controls for turret/hood/RPM
 * Distance automatically adjusts based on limelight AprilTag detection
 */
@TeleOp(name = "Competition TeleOp BLUE", group = "Competition")
public class CompetitionTeleOpBlue extends CompetitionTeleOpBase {

    public CompetitionTeleOpBlue() {
        this.alliance = AllianceColor.BLUE;
    }
}
