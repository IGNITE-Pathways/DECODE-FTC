//package org.firstinspires.ftc.teamcode.auto.OldAutos;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.auto.Pedro.Constants;
//import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
//import org.firstinspires.ftc.teamcode.core.components.Launcher;
//import org.firstinspires.ftc.teamcode.tests.individual.IntakeTransferTest;
//
//@Autonomous(name = "Blue Back Auto", group = "Auto")
//public class BlueFarOld extends OpMode {
//
//    // ==================== SHOOTING CONSTANTS (ADJUST THESE) ====================
//    // ALL SHOOTING HAPPENS AT STARTING POSITION - Adjust these based on distance to goal
//
//    // Initial Shoot (3 preloaded balls)
//    private static final double INITIAL_SHOOT_FLYWHEEL_POWER = 0.7;
//    private static final double INITIAL_SHOOT_HOOD_POSITION = 0.75;
//    private static final double INITIAL_SHOOT_TIME_SECONDS = 2.0;
//
//    // First Shoot (after picking up first set of balls, back at start)
//    private static final double SHOOT1_FLYWHEEL_POWER = 0.7;
//    private static final double SHOOT1_HOOD_POSITION = 0.75;
//    private static final double SHOOT1_TIME_SECONDS = 2.0;
//
//    // Second Shoot (after picking up second set of balls, back at start)
//    private static final double SHOOT2_FLYWHEEL_POWER = 0.7;
//    private static final double SHOOT2_HOOD_POSITION = 0.75;
//    private static final double SHOOT2_TIME_SECONDS = 2.0;
//
//    // Final Shoot (after picking up final balls and moving to final position)
//    private static final double FINAL_SHOOT_FLYWHEEL_POWER = 0.7;
//    private static final double FINAL_SHOOT_HOOD_POSITION = 0.75;
//    private static final double FINAL_SHOOT_TIME_SECONDS = 2.0;
//
//    // ==================== ROBOT COMPONENTS ====================
//    // Pedro Pathing Objects
//    private Follower follower;
//    private int pathState;
//    private PathChain getterNearestBalls, goingBackToShoot1, gettingNextSetOfBalls,
//            goingBackToShoot2, gettingFinalBalls, goingToFinalPosition;
//
//    // Robot components
//    private IntakeTransfer intakeTransfer;
//    private Launcher launcher;
//
//    // Timers
//    private Timer pathTimer;
//    private Timer opmodeTimer;
//    private Timer shootTimer;
//
//    // Starting pose
//    private final Pose startPose = new Pose(60.845, 7.910, Math.toRadians(180));
//
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        shootTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//        // Initialize robot components
//        intakeTransfer = new IntakeTransfer();
//        intakeTransfer.initialize(hardwareMap, telemetry);
//
//        launcher = new Launcher();
//        launcher.initialize(hardwareMap, telemetry);
//
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(startPose);
//    }
//
//    @Override
//    public void init_loop() {
//        telemetry.addLine("Pedro Pathing Autonomous ready!");
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        setPathState(0);
//    }
//
//    @Override
//    public void loop() {
//        // Update follower and launcher continuously
//        follower.update();
//        launcher.update();
//
//        // Run autonomous path state machine
//        autonomousPathUpdate();
//
//        // Telemetry for debugging
//        telemetry.addData("Path State", pathState);
//        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
//        telemetry.addData("Is Busy", follower.isBusy());
//        telemetry.addData("Flywheel", launcher.isSpinning() ? "ON" : "OFF");
//        telemetry.addData("Hood", "%.2f", launcher.getHoodPosition());
//        telemetry.addData("X", follower.getPose().getX());
//        telemetry.addData("Y", follower.getPose().getY());
//        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
//        telemetry.update();
//    }
//
//    @Override
//    public void stop() {
//        // Turn off all systems
//        if (launcher != null) {
//            launcher.setSpinning(false);
//        }
//        if (intakeTransfer != null) {
//            intakeTransfer.stopIntake();
//            intakeTransfer.transferDown();
//        }
//    }
//
//    private void buildPaths() {
//        // Define poses with headings
//        Pose pickup1Pose = new Pose(23.324, 35.899, Math.toRadians(180));
//        Pose pickup2Pose = new Pose(23.324, 60.034, Math.toRadians(180));
//        Pose pickup3Pose = new Pose(23.932, 83.966, Math.toRadians(180));
//        Pose finalPose = new Pose(36.710, 94.715, Math.toRadians(180));
//
//        // Path 1: Getting Nearest Balls (from starting position)
//        getterNearestBalls = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        startPose,
//                        new Pose(49.893, 37.318, Math.toRadians(180)),
//                        pickup1Pose
//                ))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        // Path 2: Going Back to Starting Shoot Position (First time)
//        goingBackToShoot1 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup1Pose, startPose))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        // Path 3: Getting Next Set of Balls
//        gettingNextSetOfBalls = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        startPose,
//                        new Pose(53.341, 60.237, Math.toRadians(180)),
//                        pickup2Pose
//                ))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        // Path 4: Going Back to Starting Shoot Position (Second time)
//        goingBackToShoot2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup2Pose, startPose))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        // Path 5: Getting Final Balls
//        gettingFinalBalls = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        startPose,
//                        new Pose(64.699, 84.169, Math.toRadians(180)),
//                        pickup3Pose
//                ))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        // Path 6: Going to Final Position
//        goingToFinalPosition = follower.pathBuilder()
//                .addPath(new BezierLine(pickup3Pose, finalPose))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//    }
//
//    private void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                // INITIAL SHOOT: Set up shooter and start shooting 3 preloaded balls
//                launcher.setPower(INITIAL_SHOOT_FLYWHEEL_POWER);
//                launcher.setHoodPosition(INITIAL_SHOOT_HOOD_POSITION);
//                launcher.setSpinning(true);
//                intakeTransfer.transferUp();
//                IntakeTransferTest.startLeftTriggerIntake(intakeTransfer);
//                shootTimer.resetTimer();
//                setPathState(1);
//                break;
//
//            case 1:
//                // Wait for initial shoot to complete
//                if (shootTimer.getElapsedTimeSeconds() >= INITIAL_SHOOT_TIME_SECONDS) {
//                    launcher.setSpinning(false);
//                    IntakeTransferTest.stopLeftTriggerIntake(intakeTransfer);
//                    intakeTransfer.transferDown();
//                    setPathState(2);
//                }
//                break;
//
//            case 2:
//                // Path 1: Getting Nearest Balls - Start intake
//                IntakeTransferTest.startLeftTriggerIntake(intakeTransfer);
//                follower.followPath(getterNearestBalls);
//                setPathState(3);
//                break;
//
//            case 3:
//                // Wait for path 1 to complete, then stop intake and start path 2
//                if (!follower.isBusy()) {
//                    IntakeTransferTest.stopLeftTriggerIntake(intakeTransfer);
//                    follower.followPath(goingBackToShoot1, true);
//                    setPathState(4);
//                }
//                break;
//
//            case 4:
//                // Path 2: Going to shoot zone - When done, start shooting
//                if (!follower.isBusy()) {
//                    launcher.setPower(SHOOT1_FLYWHEEL_POWER);
//                    launcher.setHoodPosition(SHOOT1_HOOD_POSITION);
//                    launcher.setSpinning(true);
//                    intakeTransfer.transferUp();
//                    IntakeTransferTest.startLeftTriggerIntake(intakeTransfer);
//                    shootTimer.resetTimer();
//                    setPathState(5);
//                }
//                break;
//
//            case 5:
//                // Shooting - Wait for completion
//                if (shootTimer.getElapsedTimeSeconds() >= SHOOT1_TIME_SECONDS) {
//                    launcher.setSpinning(false);
//                    IntakeTransferTest.stopLeftTriggerIntake(intakeTransfer);
//                    intakeTransfer.transferDown();
//                    // Start path 3 and intake
//                    IntakeTransferTest.startLeftTriggerIntake(intakeTransfer);
//                    follower.followPath(gettingNextSetOfBalls);
//                    setPathState(6);
//                }
//                break;
//
//            case 6:
//                // Path 3: Getting Next Set of Balls - Wait for completion
//                if (!follower.isBusy()) {
//                    IntakeTransferTest.stopLeftTriggerIntake(intakeTransfer);
//                    follower.followPath(goingBackToShoot2, true);
//                    setPathState(7);
//                }
//                break;
//
//            case 7:
//                // Path 4: Going to shoot zone - When done, start shooting
//                if (!follower.isBusy()) {
//                    launcher.setPower(SHOOT2_FLYWHEEL_POWER);
//                    launcher.setHoodPosition(SHOOT2_HOOD_POSITION);
//                    launcher.setSpinning(true);
//                    intakeTransfer.transferUp();
//                    IntakeTransferTest.startLeftTriggerIntake(intakeTransfer);
//                    shootTimer.resetTimer();
//                    setPathState(8);
//                }
//                break;
//
//            case 8:
//                // Shooting - Wait for completion
//                if (shootTimer.getElapsedTimeSeconds() >= SHOOT2_TIME_SECONDS) {
//                    launcher.setSpinning(false);
//                    IntakeTransferTest.stopLeftTriggerIntake(intakeTransfer);
//                    intakeTransfer.transferDown();
//                    // Start path 5 and intake
//                    IntakeTransferTest.startLeftTriggerIntake(intakeTransfer);
//                    follower.followPath(gettingFinalBalls);
//                    setPathState(9);
//                }
//                break;
//
//            case 9:
//                // Path 5: Getting Final Balls - Wait for completion
//                if (!follower.isBusy()) {
//                    IntakeTransferTest.stopLeftTriggerIntake(intakeTransfer);
//                    follower.followPath(goingToFinalPosition, true);
//                    setPathState(10);
//                }
//                break;
//
//            case 10:
//                // Path 6: Going to final position - When done, start final shooting
//                if (!follower.isBusy()) {
//                    launcher.setPower(FINAL_SHOOT_FLYWHEEL_POWER);
//                    launcher.setHoodPosition(FINAL_SHOOT_HOOD_POSITION);
//                    launcher.setSpinning(true);
//                    intakeTransfer.transferUp();
//                    IntakeTransferTest.startLeftTriggerIntake(intakeTransfer);
//                    shootTimer.resetTimer();
//                    setPathState(11);
//                }
//                break;
//
//            case 11:
//                // Final shooting - Wait for completion, then done
//                if (shootTimer.getElapsedTimeSeconds() >= FINAL_SHOOT_TIME_SECONDS) {
//                    launcher.setSpinning(false);
//                    IntakeTransferTest.stopLeftTriggerIntake(intakeTransfer);
//                    intakeTransfer.transferDown();
//                    setPathState(-1);
//                }
//                break;
//        }
//    }
//
//    private void setPathState(int newState) {
//        pathState = newState;
//        pathTimer.resetTimer();
//    }
//}