package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.constants.AllianceColor;

/**
 * BLUE ALLIANCE Competition TeleOp - Turret tracks AprilTag ID 20
 *
 * GP1: Drive (sticks), A = turret toggle, triggers = intake/eject
 * GP2: Y = flywheel toggle, B = lock distance preset
 */
@TeleOp(name = "Competition TeleOp BLUE", group = "Competition")
public class CompetitionTeleOpBlue extends CompetitionTeleOpBase {

    public CompetitionTeleOpBlue() {
        this.alliance = AllianceColor.BLUE;
    }
}
