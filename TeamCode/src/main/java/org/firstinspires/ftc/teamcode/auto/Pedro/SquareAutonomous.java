package org.firstinspires.ftc.teamcode.auto.Pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Simple autonomous program that moves the robot in a 3 feet square clockwise
 * Starting position: (0, 0) facing forward (0 degrees)
 * 
 * Path:
 * 1. Move forward 3 feet (36 inches) to (36, 0)
 * 2. Turn right 90° and move forward 3 feet to (36, 36)
 * 3. Turn right 90° and move forward 3 feet to (0, 36)
 * 4. Turn right 90° and move forward 3 feet back to (0, 0)
 * 5. Turn right 90° to face original direction
 */
@Autonomous(name = "Square Autonomous", group = "Simple")
public class SquareAutonomous extends LinearOpMode {
    
    private Follower follower;
    
    @Override
    public void runOpMode() {
        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        
        // Set starting pose at origin, facing forward (0 degrees = positive X direction)
        Pose startPose = new Pose(0, 0, Math.toRadians(0));
        follower.setStartingPose(startPose);
        
        // Activate all PIDFs for path following
        follower.activateAllPIDFs();
        
        telemetry.addData("Status", "Ready to run");
        telemetry.addData("Starting Position", "X: 0, Y: 0, Heading: 0°");
        telemetry.addLine("Will move in a 3 feet (36 inches) square clockwise");
        telemetry.update();
        
        waitForStart();
        
        // Execute the square path
        runSquarePath();
        
        telemetry.addLine("Square path complete!");
        telemetry.update();
    }
    
    /**
     * Execute the square path movement
     */
    private void runSquarePath() {
        // Corner 1: Move forward 3 feet (36 inches) to (36, 0), heading stays 0°
        telemetry.addLine("Moving to corner 1: (36, 0)");
        telemetry.update();
        moveToPose(new Pose(36, 0, Math.toRadians(0)));
        
        // Corner 2: Turn right 90° and move forward 3 feet to (36, 36), heading becomes 90°
        telemetry.addLine("Moving to corner 2: (36, 36)");
        telemetry.update();
        moveToPose(new Pose(36, 36, Math.toRadians(90)));
        
        // Corner 3: Turn right 90° and move forward 3 feet to (0, 36), heading becomes 180°
        telemetry.addLine("Moving to corner 3: (0, 36)");
        telemetry.update();
        moveToPose(new Pose(0, 36, Math.toRadians(180)));
        
        // Corner 4: Turn right 90° and move forward 3 feet back to (0, 0), heading becomes 270°
        telemetry.addLine("Moving to corner 4: (0, 0)");
        telemetry.update();
        moveToPose(new Pose(0, 0, Math.toRadians(270)));
        
        // Final turn: Turn right 90° to face original direction (0°)
        telemetry.addLine("Final turn: facing 0°");
        telemetry.update();
        moveToPose(new Pose(0, 0, Math.toRadians(0)));
    }
    
    /**
     * Move to a target pose using Pedro Pathing
     * @param targetPose The target pose to move to
     */
    private void moveToPose(Pose targetPose) {
        // Get current pose
        Pose currentPose = follower.getPose();
        
        // Create path using BezierLine
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                .build();
        
        // Follow the path
        follower.followPath(path);
        
        // Update follower until path is complete
        while (opModeIsActive() && (follower.isBusy() || !follower.atParametricEnd())) {
            follower.update();
            
            Pose current = follower.getPose();
            double distanceToTarget = Math.sqrt(
                Math.pow(current.getX() - targetPose.getX(), 2) + 
                Math.pow(current.getY() - targetPose.getY(), 2)
            );
            
            telemetry.addData("Current Pose", "X: %.1f, Y: %.1f, Heading: %.1f°", 
                current.getX(), current.getY(), Math.toDegrees(current.getHeading()));
            telemetry.addData("Target Pose", "X: %.1f, Y: %.1f, Heading: %.1f°", 
                targetPose.getX(), targetPose.getY(), Math.toDegrees(targetPose.getHeading()));
            telemetry.addData("Distance to Target", "%.2f inches", distanceToTarget);
            telemetry.update();
            
            sleep(10);
        }
        
        // Small delay after reaching target
        sleep(200);
    }
}
