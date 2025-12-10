package org.firstinspires.ftc.teamcode.OpModes.Main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Autonomous OpMode for Red Alliance, Left Starting Position
 */
@Autonomous(name = "Autonomous Red Left", group = "Main")
public class AutonomousRedLeft extends LinearOpMode {
    
    @Override
    public void runOpMode() {
        AutonomousMain autonomous = new AutonomousMain(
            AutonomousMain.Alliance.RED,
            AutonomousMain.StartingPosition.LEFT,
            this
        );
        
        autonomous.initialize();
        
        telemetry.addLine("Ready to start. Waiting...");
        telemetry.update();
        
        waitForStart();
        
        autonomous.run();
    }
}
