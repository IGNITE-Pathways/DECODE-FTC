package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.constants.AllianceColor;

/**
 * RED ALLIANCE Competition TeleOp - Turret tracks AprilTag ID 24
 *
 * GP1: Drive (sticks), Y = flywheel+ramp toggle, triggers = intake/eject
 * GP2: A = turret toggle, manual controls for turret/hood/RPM
 * Distance automatically adjusts based on limelight AprilTag detection
 */
@TeleOp(name = "Competition TeleOp RED", group = "Competition")
public class CompetitionTeleOpRed extends CompetitionTeleOpBase {

    public CompetitionTeleOpRed() {
        this.alliance = AllianceColor.RED;
    }
}
