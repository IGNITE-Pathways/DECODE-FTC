package org.firstinspires.ftc.teamcode.OpModes.Main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Autonomous OpMode for Blue Alliance, Right Starting Position
 */
@Autonomous(name = "Autonomous Blue Right", group = "Main")
public class AutonomousBlueRight extends LinearOpMode {
    
    @Override
    public void runOpMode() {
        AutonomousMain autonomous = new AutonomousMain(
            AutonomousMain.Alliance.BLUE,
            AutonomousMain.StartingPosition.RIGHT,
            this
        );
        
        autonomous.initialize();
        
        telemetry.addLine("Ready to start. Waiting...");
        telemetry.update();
        
        waitForStart();
        
        autonomous.run();
    }
}
