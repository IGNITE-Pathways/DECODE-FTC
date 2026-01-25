package org.firstinspires.ftc.teamcode.auto.Pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Autonomous program that follows the trajectory path from trajectory (8).pp
 * Starting position: (56, 8) facing 90 degrees
 * Structured like Pedro Pathing example
 */
@Autonomous(name = "Trajectory Autonomous", group = "Simple")
public class TrajectoryAutonomous extends LinearOpMode {
    
    private Follower follower;
    private int pathState;
    
    // Poses from trajectory (8).pp
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose seg1End = new Pose(56, 12, Math.toRadians(115));
    private final Pose seg3End = new Pose(40, 39, Math.toRadians(180));
    private final Pose seg4End = new Pose(35, 39, Math.toRadians(180));
    private final Pose seg6End = new Pose(29, 39, Math.toRadians(180));
    private final Pose seg8End = new Pose(22, 39, Math.toRadians(180));
    private final Pose seg10End = new Pose(56, 12, Math.toRadians(115));
    private final Pose seg13End = new Pose(40, 65, Math.toRadians(180));
    private final Pose seg14End = new Pose(35, 65, Math.toRadians(180));
    private final Pose seg16End = new Pose(29, 65, Math.toRadians(180));
    private final Pose seg18End = new Pose(22, 65, Math.toRadians(180));
    private final Pose seg20End = new Pose(56, 110, Math.toRadians(150));
    private final Pose seg23End = new Pose(56, 50, Math.toRadians(150));
    
    // Path chains - removing duplicate coordinates
    private PathChain path1; // align: (56,8) to (56,12)
    private PathChain path2; // align with set 1: (56,12) to (40,39)
    private PathChain path3; // get ball 1: (40,39) to (35,39)
    private PathChain path4; // get ball 2: (35,39) to (29,39)
    private PathChain path5; // get ball 3: (29,39) to (22,39)
    private PathChain path6; // reset to shoot: (22,39) to (56,12)
    private PathChain path7; // align with set 2: (56,12) to (40,65)
    private PathChain path8; // get ball 1: (40,65) to (35,65)
    private PathChain path9; // get ball 2: (35,65) to (29,65)
    private PathChain path10; // get ball 3: (29,65) to (22,65)
    private PathChain path11; // reset to shoot: (22,65) to (56,110)
    private PathChain path12; // final path: (56,110) to (56,50)
    
    @Override
    public void runOpMode() {
        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.activateAllPIDFs();
        
        // Build all paths
        buildPaths();
        
        telemetry.addData("Status", "Ready to run");
        telemetry.addData("Starting Position", "X: 56, Y: 8, Heading: 90°");
        telemetry.update();
        
        waitForStart();
        
        // Start with path state 0
        pathState = 0;
        
        // Main loop - update follower and manage path states
        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();
            
            // Telemetry
            Pose current = follower.getPose();
            telemetry.addData("Path State", pathState);
            telemetry.addData("X", current.getX());
            telemetry.addData("Y", current.getY());
            telemetry.addData("Heading", Math.toDegrees(current.getHeading()));
            telemetry.addData("Is Busy", follower.isBusy());
            telemetry.update();
        }
    }
    
    /**
     * Build all paths before autonomous starts
     */
    private void buildPaths() {
        // Path 1: align - (56,8) to (56,12), linear heading 90→115
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, seg1End))
                .setLinearHeadingInterpolation(startPose.getHeading(), seg1End.getHeading())
                .build();
        
        // Path 2: align with set 1 - (56,12) to (40,39), linear heading 115→180
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(seg1End, seg3End))
                .setLinearHeadingInterpolation(seg1End.getHeading(), seg3End.getHeading())
                .build();
        
        // Path 3: get ball 1 - (40,39) to (35,39), tangential heading
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(seg3End, seg4End))
                .setTangentHeadingInterpolation()
                .build();
        
        // Path 4: get ball 2 - (35,39) to (29,39), tangential heading
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(seg4End, seg6End))
                .setTangentHeadingInterpolation()
                .build();
        
        // Path 5: get ball 3 - (29,39) to (22,39), tangential heading
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(seg6End, seg8End))
                .setTangentHeadingInterpolation()
                .build();
        
        // Path 6: reset to shoot - (22,39) to (56,12), linear heading 180→115
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(seg8End, seg10End))
                .setLinearHeadingInterpolation(seg8End.getHeading(), seg10End.getHeading())
                .build();
        
        // Path 7: align with set 2 - (56,12) to (40,65), linear heading 115→180
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(seg10End, seg13End))
                .setLinearHeadingInterpolation(seg10End.getHeading(), seg13End.getHeading())
                .build();
        
        // Path 8: get ball 1 - (40,65) to (35,65), tangential heading
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(seg13End, seg14End))
                .setTangentHeadingInterpolation()
                .build();
        
        // Path 9: get ball 2 - (35,65) to (29,65), tangential heading
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(seg14End, seg16End))
                .setTangentHeadingInterpolation()
                .build();
        
        // Path 10: get ball 3 - (29,65) to (22,65), tangential heading
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(seg16End, seg18End))
                .setTangentHeadingInterpolation()
                .build();
        
        // Path 11: reset to shoot - (22,65) to (56,110), linear heading 180→150
        path11 = follower.pathBuilder()
                .addPath(new BezierLine(seg18End, seg20End))
                .setLinearHeadingInterpolation(seg18End.getHeading(), seg20End.getHeading())
                .build();
        
        // Path 12: final path - (56,110) to (56,50), linear heading 150→150
        path12 = follower.pathBuilder()
                .addPath(new BezierLine(seg20End, seg23End))
                .setLinearHeadingInterpolation(seg20End.getHeading(), seg23End.getHeading())
                .build();
    }
    
    /**
     * State machine to manage path progression
     */
    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(path2);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(path3);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(path4);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(path5);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(path6);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(path7);
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(path8);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(path9);
                    pathState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(path10);
                    pathState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(path11);
                    pathState = 11;
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(path12);
                    pathState = 12;
                }
                break;
            case 12:
                // All paths complete
                if (!follower.isBusy()) {
                    pathState = -1; // Stop state
                }
                break;
        }
    }
}
