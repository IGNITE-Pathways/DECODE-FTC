package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
import org.firstinspires.ftc.teamcode.tests.individual.IntakeTransferTest;

@Autonomous(name = "Blue Back Auto", group = "Auto")
public class BlueBack extends OpMode {

    // Pedro Pathing Objects
    private Follower follower;
    private int pathState;
    private PathChain getterNearestBalls, goingBackToShoot1, gettingNextSetOfBalls,
            goingBackToShoot2, gettingFinalBalls, goingToFinalPosition;

    // IntakeTransfer component
    private IntakeTransfer intakeTransfer;

    // Timers
    private Timer pathTimer;
    private Timer opmodeTimer;
    private Timer ejectTimer;

    // Starting pose
    private final Pose startPose = new Pose(60.845, 7.910, Math.toRadians(180));

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        ejectTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initialize IntakeTransfer
        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Pedro Pathing Autonomous ready!");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        // Update follower continuously
        follower.update();

        // Run autonomous path state machine
        autonomousPathUpdate();

        // Telemetry for debugging
        telemetry.addData("Path State", pathState);
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Is Busy", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        // Autonomous complete
    }

    private void buildPaths() {
        // Define poses with headings
        Pose pickup1Pose = new Pose(23.324, 35.899, Math.toRadians(180));
        Pose shoot1Pose = new Pose(62.265, 24.135, Math.toRadians(180));
        Pose pickup2Pose = new Pose(23.324, 60.034, Math.toRadians(180));
        Pose shoot2Pose = new Pose(62.468, 23.932, Math.toRadians(180));
        Pose pickup3Pose = new Pose(23.932, 83.966, Math.toRadians(180));
        Pose finalPose = new Pose(36.710, 94.715, Math.toRadians(180));

        // Path 1: Getting Nearest Balls
        getterNearestBalls = follower.pathBuilder()
                .addPath(new BezierCurve(
                        startPose,
                        new Pose(49.893, 37.318, Math.toRadians(180)),
                        pickup1Pose
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 2: Going Back to Shoot (First time)
        goingBackToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, shoot1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 3: Getting Next Set of Balls
        gettingNextSetOfBalls = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shoot1Pose,
                        new Pose(53.341, 60.237, Math.toRadians(180)),
                        pickup2Pose
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 4: Going Back to Shoot (Second time)
        goingBackToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, shoot2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 5: Getting Final Balls
        gettingFinalBalls = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shoot2Pose,
                        new Pose(64.699, 84.169, Math.toRadians(180)),
                        pickup3Pose
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 6: Going to Final Position
        goingToFinalPosition = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, finalPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Path 1: Getting Nearest Balls - Start intake
                IntakeTransferTest.startLeftTriggerIntake(intakeTransfer);
                follower.followPath(getterNearestBalls);
                setPathState(1);
                break;

            case 1:
                // Wait for path 1 to complete, then stop intake and start path 2
                if (!follower.isBusy()) {
                    IntakeTransferTest.stopLeftTriggerIntake(intakeTransfer);
                    follower.followPath(goingBackToShoot1, true);
                    setPathState(2);
                }
                break;

            case 2:
                // Path 2: Going to shoot zone - When done, start ejecting
                if (!follower.isBusy()) {
                    IntakeTransferTest.startRightTriggerEject(intakeTransfer);
                    ejectTimer.resetTimer();
                    setPathState(3);
                }
                break;

            case 3:
                // Ejecting for 2 seconds
                if (ejectTimer.getElapsedTimeSeconds() >= 2.0) {
                    IntakeTransferTest.stopRightTriggerEject(intakeTransfer);
                    // Start path 3 and intake
                    IntakeTransferTest.startLeftTriggerIntake(intakeTransfer);
                    follower.followPath(gettingNextSetOfBalls);
                    setPathState(4);
                }
                break;

            case 4:
                // Path 3: Getting Next Set of Balls - Wait for completion
                if (!follower.isBusy()) {
                    IntakeTransferTest.stopLeftTriggerIntake(intakeTransfer);
                    follower.followPath(goingBackToShoot2, true);
                    setPathState(5);
                }
                break;

            case 5:
                // Path 4: Going to shoot zone - When done, start ejecting
                if (!follower.isBusy()) {
                    IntakeTransferTest.startRightTriggerEject(intakeTransfer);
                    ejectTimer.resetTimer();
                    setPathState(6);
                }
                break;

            case 6:
                // Ejecting for 2 seconds
                if (ejectTimer.getElapsedTimeSeconds() >= 2.0) {
                    IntakeTransferTest.stopRightTriggerEject(intakeTransfer);
                    // Start path 5 and intake
                    IntakeTransferTest.startLeftTriggerIntake(intakeTransfer);
                    follower.followPath(gettingFinalBalls);
                    setPathState(7);
                }
                break;

            case 7:
                // Path 5: Getting Final Balls - Wait for completion
                if (!follower.isBusy()) {
                    IntakeTransferTest.stopLeftTriggerIntake(intakeTransfer);
                    follower.followPath(goingToFinalPosition, true);
                    setPathState(8);
                }
                break;

            case 8:
                // Path 6: Going to final position - When done, start ejecting
                if (!follower.isBusy()) {
                    IntakeTransferTest.startRightTriggerEject(intakeTransfer);
                    ejectTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9:
                // Ejecting for 2 seconds, then done
                if (ejectTimer.getElapsedTimeSeconds() >= 2.0) {
                    IntakeTransferTest.stopRightTriggerEject(intakeTransfer);
                    setPathState(-1);
                }
                break;
        }
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
}