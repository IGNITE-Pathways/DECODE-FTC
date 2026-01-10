package org.firstinspires.ftc.teamcode.OpModes.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.Robot;

@Autonomous(name = "Autonomous - Ball Collection 2", group = "Autonomous")
public class SamarBlueRight extends OpMode {
    private Follower follower;
    private Robot robot;
    private Timer pathTimer, opmodeTimer, waitTimer;

    private int pathState;
    private boolean isWaiting = false;
    private static final double WAIT_TIME_SECONDS = 1.5;
    private static final double BALL_WAIT_TIME_SECONDS = 2.0;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));

    // Path declarations
    private PathChain Path1, Path2, Path3, Path4, Path5, Path6;
    private PathChain Path7, Path8, Path9, Path10, Path11, Path12;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        waitTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        robot = new Robot();
        robot.initialize(hardwareMap, telemetry, null);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized");
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

        telemetry.addData("path state", pathState);
        telemetry.addData("is waiting", isWaiting);
        telemetry.addData("is busy", follower.isBusy());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        if (pathState >= 2 && pathState <= 4) {
            telemetry.addLine("Getting balls - Path " + (pathState - 1));
        }
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        if (isWaiting) {
            double currentWaitTime = (pathState >= 2 && pathState <= 4) || (pathState >= 7 && pathState <= 9)
                    ? BALL_WAIT_TIME_SECONDS : WAIT_TIME_SECONDS;

            if (waitTimer.getElapsedTimeSeconds() >= currentWaitTime) {
                isWaiting = false;
                waitTimer.resetTimer();
            } else {
                return;
            }
        }

        switch (pathState) {
            case 0:
                follower.followPath(Path1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    robot.getIntakeTransfer().startIntake();
                    startWait();
                    follower.followPath(Path2);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    startWait();
                    follower.followPath(Path3);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    startWait();
                    follower.followPath(Path4);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    startWait();
                    follower.followPath(Path5);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    robot.getIntakeTransfer().stopIntake();
                    startWait();
                    follower.followPath(Path6);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    robot.getIntakeTransfer().startIntake();
                    startWait();
                    follower.followPath(Path7);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    startWait();
                    follower.followPath(Path8);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    startWait();
                    follower.followPath(Path9);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    startWait();
                    follower.followPath(Path10);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    robot.getIntakeTransfer().stopIntake();
                    startWait();
                    follower.followPath(Path11);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    startWait();
                    follower.followPath(Path12);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    private void startWait() {
        isWaiting = true;
        waitTimer.resetTimer();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void buildPaths() {
        Path1 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(56, 8, Math.toRadians(90)), new Pose(56, 12, Math.toRadians(115))))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                .build();

        Path2 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(56, 12, Math.toRadians(115)), new Pose(40, 39, Math.toRadians(180))))
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .build();

        Path3 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(40, 39, Math.toRadians(180)), new Pose(35, 39, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        Path4 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(35, 39, Math.toRadians(180)), new Pose(29, 39, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        Path5 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(29, 39, Math.toRadians(180)), new Pose(22, 39, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        Path6 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(22, 39, Math.toRadians(180)), new Pose(56, 12, Math.toRadians(115))))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                .build();

        Path7 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(56, 12, Math.toRadians(115)), new Pose(40, 65, Math.toRadians(180))))
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .build();

        Path8 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(40, 65, Math.toRadians(180)), new Pose(35, 65, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        Path9 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(35, 65, Math.toRadians(180)), new Pose(29, 65, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        Path10 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(29, 65, Math.toRadians(180)), new Pose(22, 65, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();

        Path11 = follower.pathBuilder()
                .setGlobalDeceleration(2.0)
                .addPath(new BezierLine(new Pose(22, 65, Math.toRadians(180)), new Pose(56, 110, Math.toRadians(150))))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150))
                .build();

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
