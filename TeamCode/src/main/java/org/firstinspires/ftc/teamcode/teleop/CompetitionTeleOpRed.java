package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.constants.AllianceColor;

/**
 * RED ALLIANCE Competition TeleOp - Turret tracks AprilTag ID 24
 *
 * GP1: Drive (sticks), A = turret toggle, triggers = intake/eject
 * GP2: Y = flywheel toggle, B = lock distance preset
 */
@TeleOp(name = "Competition TeleOp RED", group = "Competition")
public class CompetitionTeleOpRed extends CompetitionTeleOpBase {

    public CompetitionTeleOpRed() {
        this.alliance = AllianceColor.RED;
    }
}
