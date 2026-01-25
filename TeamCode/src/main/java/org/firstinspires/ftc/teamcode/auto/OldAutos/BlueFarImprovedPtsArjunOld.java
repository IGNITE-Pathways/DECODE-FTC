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
//
///**
// * Autonomous OpMode using Pedro Pathing
// * Executes multiple ball collection sequences and shooting cycles
// */
//@Autonomous(name = "Blue Far Improved Pts Arjun", group = "Autonomous")
//public class BlueFarImprovedPtsArjunOld extends OpMode {
//    private Follower follower;
//    private Paths paths;
//    private Timer autonomousTimer;
//
//    // State machine for autonomous sequences
//    private enum AutoState {
//        START,
//        GOING_TO_NEAREST_BALLS,
//        GETTING_FIRST_BALL_SET_1,
//        GETTING_SECOND_BALL_SET_1,
//        GETTING_THIRD_BALL_SET_1,
//        GOING_BACK_TO_SHOOT_1,
//        GOING_TO_NEXT_SET,
//        GETTING_FIRST_BALL_SET_2,
//        GETTING_SECOND_BALL_SET_2,
//        GETTING_THIRD_BALL_SET_2,
//        GOING_BACK_TO_SHOOT_2,
//        GOING_TO_THIRD_SET,
//        GETTING_FIRST_BALL_SET_3,
//        GETTING_SECOND_BALL_SET_3,
//        GETTING_THIRD_BALL_SET_3,
//        FINAL_PATH,
//        IDLE
//    }
//
//    private AutoState currentState = AutoState.START;
//
//    @Override
//    public void init() {
//        // Initialize Pedro Pathing follower using Constants
//        follower = Constants.createFollower(hardwareMap);
//
//        // Build all paths
//        paths = new Paths(follower);
//
//        // Set initial pose
//        follower.setStartingPose(new Pose(60.845, 7.910, Math.toRadians(180)));
//
//        telemetry.addData("Status", "Initialized - Ready to Start");
//        telemetry.update();
//    }
//
//    @Override
//    public void init_loop() {
//        // Optional: Display field status or calibration info
//    }
//
//    @Override
//    public void start() {
//        autonomousTimer = new Timer();
//        autonomousTimer.resetTimer();
//        currentState = AutoState.START;
//        telemetry.addData("Status", "Autonomous Started");
//        telemetry.update();
//    }
//
//    @Override
//    public void loop() {
//        // Update follower
//        follower.update();
//
//        // Execute state machine
//        switch (currentState) {
//            case START:
//                // Begin first sequence
//                follower.followPath(paths.getGoingToNearestBalls());
//                currentState = AutoState.GOING_TO_NEAREST_BALLS;
//                break;
//
//            case GOING_TO_NEAREST_BALLS:
//                if (!follower.isBusy()) {
//                    // Call intake method here if you have one
//                    // robot.intake.turnOn();
//                    follower.followPath(paths.getGettingFirstBall_Set1());
//                    currentState = AutoState.GETTING_FIRST_BALL_SET_1;
//                }
//                break;
//
//            case GETTING_FIRST_BALL_SET_1:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.getGettingSecondBall_Set1());
//                    currentState = AutoState.GETTING_SECOND_BALL_SET_1;
//                }
//                break;
//
//            case GETTING_SECOND_BALL_SET_1:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.getGettingThirdBall_Set1());
//                    currentState = AutoState.GETTING_THIRD_BALL_SET_1;
//                }
//                break;
//
//            case GETTING_THIRD_BALL_SET_1:
//                if (!follower.isBusy()) {
//                    // robot.intake.turnOff();
//                    follower.followPath(paths.getGoingBacktoShoot_Set1());
//                    currentState = AutoState.GOING_BACK_TO_SHOOT_1;
//                }
//                break;
//
//            case GOING_BACK_TO_SHOOT_1:
//                if (!follower.isBusy()) {
//                    // Execute shooting sequence here
//                    // robot.shooter.shoot();
//                    // Sleep(500); // Wait for shot to complete
//                    follower.followPath(paths.getGettingNextSetofBalls());
//                    currentState = AutoState.GOING_TO_NEXT_SET;
//                }
//                break;
//
//            case GOING_TO_NEXT_SET:
//                if (!follower.isBusy()) {
//                    // robot.intake.turnOn();
//                    follower.followPath(paths.getGettingFirstBall_Set2());
//                    currentState = AutoState.GETTING_FIRST_BALL_SET_2;
//                }
//                break;
//
//            case GETTING_FIRST_BALL_SET_2:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.getGettingSecondBall_Set2());
//                    currentState = AutoState.GETTING_SECOND_BALL_SET_2;
//                }
//                break;
//
//            case GETTING_SECOND_BALL_SET_2:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.getGettingThirdBall_Set2());
//                    currentState = AutoState.GETTING_THIRD_BALL_SET_2;
//                }
//                break;
//
//            case GETTING_THIRD_BALL_SET_2:
//                if (!follower.isBusy()) {
//                    // robot.intake.turnOff();
//                    follower.followPath(paths.getGoingBacktoShoot_Set2());
//                    currentState = AutoState.GOING_BACK_TO_SHOOT_2;
//                }
//                break;
//
//            case GOING_BACK_TO_SHOOT_2:
//                if (!follower.isBusy()) {
//                    // Execute shooting sequence
//                    // robot.shooter.shoot();
//                    // Sleep(500);
//                    follower.followPath(paths.getGettingThirdSetofBalls());
//                    currentState = AutoState.GOING_TO_THIRD_SET;
//                }
//                break;
//
//            case GOING_TO_THIRD_SET:
//                if (!follower.isBusy()) {
//                    // robot.intake.turnOn();
//                    follower.followPath(paths.getGettingFirstBall_Set3());
//                    currentState = AutoState.GETTING_FIRST_BALL_SET_3;
//                }
//                break;
//
//            case GETTING_FIRST_BALL_SET_3:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.getGettingSecondBall_Set3());
//                    currentState = AutoState.GETTING_SECOND_BALL_SET_3;
//                }
//                break;
//
//            case GETTING_SECOND_BALL_SET_3:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.getGettingThirdBall_Set3());
//                    currentState = AutoState.GETTING_THIRD_BALL_SET_3;
//                }
//                break;
//
//            case GETTING_THIRD_BALL_SET_3:
//                if (!follower.isBusy()) {
//                    // robot.intake.turnOff();
//                    follower.followPath(paths.getPath15());
//                    currentState = AutoState.FINAL_PATH;
//                }
//                break;
//
//            case FINAL_PATH:
//                if (!follower.isBusy()) {
//                    // Final shooting sequence
//                    // robot.shooter.shoot();
//                    currentState = AutoState.IDLE;
//                }
//                break;
//
//            case IDLE:
//                // Autonomous complete
//                break;
//        }
//
//        // Telemetry
//        telemetry.addData("State", currentState);
//        telemetry.addData("Follower Busy", follower.isBusy());
//        Pose currentPose = follower.getPose();
//        telemetry.addData("X", "%.1f", currentPose.getX());
//        telemetry.addData("Y", "%.1f", currentPose.getY());
//        telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
//        telemetry.addData("Time Elapsed", String.format("%.1f s", autonomousTimer.getElapsedTimeSeconds()));
//        telemetry.update();
//    }
//
//    @Override
//    public void stop() {
//        follower.breakFollowing();
//    }
//
//    /**
//     * Paths class containing all PathChain definitions
//     * This mirrors your Pedro Pathing Visualizer output
//     */
//    public static class Paths {
//        // First set of ball collection paths
//        private PathChain GoingToNearestBalls;
//        private PathChain GettingFirstBall_Set1;
//        private PathChain GettingSecondBall_Set1;
//        private PathChain GettingThirdBall_Set1;
//        private PathChain GoingBacktoShoot_Set1;
//
//        // Second set of ball collection paths
//        private PathChain GettingNextSetofBalls;
//        private PathChain GettingFirstBall_Set2;
//        private PathChain GettingSecondBall_Set2;
//        private PathChain GettingThirdBall_Set2;
//        private PathChain GoingBacktoShoot_Set2;
//
//        // Third set of ball collection paths
//        private PathChain GettingThirdSetofBalls;
//        private PathChain GettingFirstBall_Set3;
//        private PathChain GettingSecondBall_Set3;
//        private PathChain GettingThirdBall_Set3;
//        private PathChain Path15;
//
//        public Paths(Follower follower) {
//            // ===== FIRST SET OF BALLS =====
//            GoingToNearestBalls = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(60.845, 7.910),
//                            new Pose(41.172, 36.034)
//                    ))
//                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                    .build();
//
//            GettingFirstBall_Set1 = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(41.172, 36.034),
//                            new Pose(35.392, 35.899)
//                    ))
//                    .setTangentHeadingInterpolation()
//                    .build();
//
//            GettingSecondBall_Set1 = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(35.392, 35.899),
//                            new Pose(29.358, 35.899)
//                    ))
//                    .setTangentHeadingInterpolation()
//                    .build();
//
//            GettingThirdBall_Set1 = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(29.358, 35.899),
//                            new Pose(22.487, 35.899)
//                    ))
//                    .setTangentHeadingInterpolation()
//                    .build();
//
//            GoingBacktoShoot_Set1 = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(22.487, 35.899),
//                            new Pose(60.845, 7.910)
//                    ))
//                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                    .build();
//
//            // ===== SECOND SET OF BALLS =====
//            GettingNextSetofBalls = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Pose(60.845, 7.910),
//                            new Pose(53.341, 60.237),
//                            new Pose(39.752, 59.831)
//                    ))
//                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                    .build();
//
//            GettingFirstBall_Set2 = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(39.752, 59.831),
//                            new Pose(34.580, 59.730)
//                    ))
//                    .setTangentHeadingInterpolation()
//                    .build();
//
//            GettingSecondBall_Set2 = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(34.580, 59.730),
//                            new Pose(30.068, 59.780)
//                    ))
//                    .setTangentHeadingInterpolation()
//                    .build();
//
//            GettingThirdBall_Set2 = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(30.068, 59.780),
//                            new Pose(24.566, 60.008)
//                    ))
//                    .setTangentHeadingInterpolation()
//                    .build();
//
//            GoingBacktoShoot_Set2 = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(24.566, 60.008),
//                            new Pose(60.845, 7.910)
//                    ))
//                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                    .build();
//
//            // ===== THIRD SET OF BALLS =====
//            GettingThirdSetofBalls = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Pose(60.845, 7.910),
//                            new Pose(59.020, 81.938),
//                            new Pose(40.158, 84.169)
//                    ))
//                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                    .build();
//
//            GettingFirstBall_Set3 = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(40.158, 84.169),
//                            new Pose(34.885, 84.169)
//                    ))
//                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                    .build();
//
//            GettingSecondBall_Set3 = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(34.885, 84.169),
//                            new Pose(30.070, 84.118)
//                    ))
//                    .setTangentHeadingInterpolation()
//                    .build();
//
//            GettingThirdBall_Set3 = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(30.070, 84.118),
//                            new Pose(24.513, 83.989)
//                    ))
//                    .setTangentHeadingInterpolation()
//                    .build();
//
//            Path15 = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Pose(24.513, 83.989),
//                            new Pose(47.587, 95.083),
//                            new Pose(40.994, 101.603)
//                    ))
//                    .setTangentHeadingInterpolation()
//                    .build();
//        }
//
//        // Getters for all paths
//        public PathChain getGoingToNearestBalls() { return GoingToNearestBalls; }
//        public PathChain getGettingFirstBall_Set1() { return GettingFirstBall_Set1; }
//        public PathChain getGettingSecondBall_Set1() { return GettingSecondBall_Set1; }
//        public PathChain getGettingThirdBall_Set1() { return GettingThirdBall_Set1; }
//        public PathChain getGoingBacktoShoot_Set1() { return GoingBacktoShoot_Set1; }
//        public PathChain getGettingNextSetofBalls() { return GettingNextSetofBalls; }
//        public PathChain getGettingFirstBall_Set2() { return GettingFirstBall_Set2; }
//        public PathChain getGettingSecondBall_Set2() { return GettingSecondBall_Set2; }
//        public PathChain getGettingThirdBall_Set2() { return GettingThirdBall_Set2; }
//        public PathChain getGoingBacktoShoot_Set2() { return GoingBacktoShoot_Set2; }
//        public PathChain getGettingThirdSetofBalls() { return GettingThirdSetofBalls; }
//        public PathChain getGettingFirstBall_Set3() { return GettingFirstBall_Set3; }
//        public PathChain getGettingSecondBall_Set3() { return GettingSecondBall_Set3; }
//        public PathChain getGettingThirdBall_Set3() { return GettingThirdBall_Set3; }
//        public PathChain getPath15() { return Path15; }
//    }
//}