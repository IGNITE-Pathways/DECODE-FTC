package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.core.constants.AllianceColor;
import org.firstinspires.ftc.teamcode.core.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.core.components.Robot;

/**
 * Base Autonomous class - DO NOT RUN DIRECTLY
 * Use AutoOpMainBlue or AutoOpMainRed instead
 *
 * Uses Pedro Pathing for trajectory following.
 */
public class AutoOpMain extends OpMode {
    private Follower follower;
    protected Robot robot;
    private Timer pathTimer, opmodeTimer, waitTimer;

    private int pathState;
    private boolean isWaiting = false;
    private double customWaitTime = 1.5;
    private static final double WAIT_TIME_SECONDS = 1.5;
    private static final double FLYWHEEL_SPINUP_TIME = 1.5;
    private static final double SHOOTING_TIME = 3.0;

    // Alliance color
    protected AllianceColor allianceColor = null;

    protected void setAllianceColor(AllianceColor color) {
        this.allianceColor = color;
    }

    // Starting position
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));

    // Path declarations
    private PathChain Path1; // align: (56,8) to (56,12) - BLUE alliance
    private PathChain Path1Red; // align: (88,8) to (88,12) - RED alliance (mirrored from Path1)
    private PathChain Path2; // align with set 1: (56,12) to (40,39)
    private PathChain Path3; // get ball 1: (40,39) to (35,39)
    private PathChain Path4; // get ball 2: (35,39) to (29,39)
    private PathChain Path5; // get ball 3: (29,39) to (22,39)
    private PathChain Path6; // reset to shoot: (22,39) to (56,12)
    private PathChain Path7; // align with set 2: (56,12) to (40,65)
    private PathChain Path8; // get ball 1: (40,65) to (35,65)
    private PathChain Path9; // get ball 2: (35,65) to (29,65)
    private PathChain Path10; // get ball 3: (29,65) to (22,65)
    private PathChain Path11; // reset to shoot: (22,65) to (56,110)
    private PathChain Path12; // final path: (56,110) to (56,50)

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        waitTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        robot = new Robot();
        robot.initialize(hardwareMap, telemetry, null, allianceColor);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized");
        if (allianceColor != null) {
            telemetry.addData("Alliance", allianceColor.name());
        }
        telemetry.addLine("");
        telemetry.addLine("Ready to start!");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Telemetry
        telemetry.addData("Path State", pathState);
        telemetry.addData("Is Waiting", isWaiting);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("X", "%.1f", follower.getPose().getX());
        telemetry.addData("Y", "%.1f", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        // Handle waiting
        if (isWaiting) {
            if (waitTimer.getElapsedTimeSeconds() >= customWaitTime) {
                isWaiting = false;
                waitTimer.resetTimer();
                customWaitTime = WAIT_TIME_SECONDS;
            } else {
                return;
            }
        }

        switch (pathState) {
            case 0: // Start - follow initial path
                if (allianceColor == AllianceColor.RED) {
                    follower.followPath(Path1Red);
                } else {
                    follower.followPath(Path1);
                }
                setPathState(1);
                break;

            case 1: // Wait for path, then start shooting prep
                if (!follower.isBusy()) {
                    // Start flywheel spin-up
                    robot.setFlywheelPower(RobotConstants.FLYWHEEL_SHOOTING_POWER);
                    robot.startFlywheel();
                    robot.setHoodPosition(RobotConstants.HOOD_DEFAULT_POSITION);

                    // Lock turret position
                    robot.getTurret().setPositionDirect(0.6);

                    setPathState(2);
                }
                break;

            case 2: // Wait for flywheel spin-up
                if (pathTimer.getElapsedTimeSeconds() >= FLYWHEEL_SPINUP_TIME) {
                    // Start shooting sequence
                    robot.getIntakeTransfer().transferUp();
                    robot.getIntakeTransfer().startIntake();
                    setPathState(3);
                }
                break;

            case 3: // Shooting - wait for balls to fire
                if (pathTimer.getElapsedTimeSeconds() >= SHOOTING_TIME) {
                    // Stop shooting
                    robot.getIntakeTransfer().stopIntake();
                    robot.getIntakeTransfer().transferDown();
                    robot.stopFlywheel();
                    setPathState(4);
                }
                break;

            case 4: // Follow Path2 to next position
                if (!follower.isBusy()) {
                    follower.followPath(Path2);
                    setPathState(5);
                }
                break;

            case 5: // Wait for Path2 to complete
                if (!follower.isBusy()) {
                    setPathState(-1); // Done
                }
                break;

            default:
                // Autonomous complete
                break;
        }
    }

    private void startWait(double waitTime) {
        isWaiting = true;
        customWaitTime = waitTime;
        waitTimer.resetTimer();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void buildPaths() {
        // Path 1: align - (56,8) to (56,12), linear heading 90→115 (BLUE alliance)
        Path1 = follower.pathBuilder()
                .setGlobalDeceleration(2.0) // Higher deceleration slows paths down significantly
                .addPath(new BezierLine(new Pose(56, 8, Math.toRadians(90)), new Pose(56, 12, Math.toRadians(115))))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                .build();

        // Path 1Red: align - (88,8) to (88,12), linear heading 90→65 (RED alliance, mirrored from Path1)
        // Mirroring left-to-right: x -> 144 - x, heading -> 180° - heading
        Path1Red = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(88, 8, Math.toRadians(90)), new Pose(88, 12, Math.toRadians(65))))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))
                .build();

        // Path 2: align with set 1 - (56,12) to (40,39), linear heading 115→180
        Path2 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(56, 12, Math.toRadians(115)), new Pose(40, 39, Math.toRadians(180))))
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .build();

        // Path 3: get ball 1 - (40,39) to (35,39), tangential heading
        Path3 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(40, 39, Math.toRadians(180)), new Pose(35, 39, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        // Path 4: get ball 2 - (35,39) to (29,39), tangential heading
        Path4 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(35, 39, Math.toRadians(180)), new Pose(29, 39, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        // Path 5: get ball 3 - (29,39) to (22,39), tangential heading
        Path5 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(29, 39, Math.toRadians(180)), new Pose(22, 39, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        // Path 6: reset to shoot - (22,39) to (56,12), linear heading 180→115
        Path6 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(22, 39, Math.toRadians(180)), new Pose(56, 12, Math.toRadians(115))))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                .build();

        // Path 7: align with set 2 - (56,12) to (40,65), linear heading 115→180
        Path7 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(56, 12, Math.toRadians(115)), new Pose(40, 65, Math.toRadians(180))))
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .build();

        // Path 8: get ball 1 - (40,65) to (35,65), tangential heading
        Path8 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(40, 65, Math.toRadians(180)), new Pose(35, 65, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        // Path 9: get ball 2 - (35,65) to (29,65), tangential heading
        Path9 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(35, 65, Math.toRadians(180)), new Pose(29, 65, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        // Path 10: get ball 3 - (29,65) to (22,65), tangential heading
        Path10 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(29, 65, Math.toRadians(180)), new Pose(22, 65, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        // Path 11: reset to shoot - (22,65) to (56,110), linear heading 180→150
        Path11 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(22, 65, Math.toRadians(180)), new Pose(56, 110, Math.toRadians(150))))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150))
                .build();

        // Path 12: final path - (56,110) to (56,50), linear heading 150→150
        Path12 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(56, 110, Math.toRadians(150)), new Pose(56, 50, Math.toRadians(150))))
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(150))
                .build();
    }

    @Override
    public void stop() {
        robot.stopAll();
    }
}
