package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
import org.firstinspires.ftc.teamcode.core.components.Launcher;
import com.pedropathing.util.Timer;

/**
 * Centralized Shooting Configuration System
 */
public class ShootingFunction {

    public enum AutonPath {
        BLUE_FAR,
        BLUE_NEAR,
        RED_FAR,
        RED_NEAR
    }

    public enum ShootingPosition {
        PRELOAD,
        SET_1,
        SET_2
    }

    public static class Configuration {
        public final double flywheelPower;
        public final double hoodPosition;
        public final double turretPosition;
        public final double shootTimeSeconds;
        public final TimingConfig timing;

        public Configuration(double flywheelPower, double hoodPosition, double turretPosition,
                             double shootTimeSeconds, TimingConfig timing) {
            this.flywheelPower = flywheelPower;
            this.hoodPosition = hoodPosition;
            this.turretPosition = turretPosition;
            this.shootTimeSeconds = shootTimeSeconds;
            this.timing = timing;
        }
    }

    public static class TimingConfig {
        public final double spinupTime;
        public final double ball1FeedTime;
        public final double ball1RecoveryTime;
        public final double ball2FeedTime;
        public final double ball2RecoveryTime;
        public final double ball3FeedTime;

        public final boolean spinupIntakeOn;
        public final boolean ball1FeedIntakeOn;
        public final boolean ball1RecoveryIntakeOn;
        public final boolean ball2FeedIntakeOn;
        public final boolean ball2RecoveryIntakeOn;
        public final boolean ball3FeedIntakeOn;
        public final boolean finishIntakeOn;

        public final boolean spinupEjectOn;
        public final boolean ball1FeedEjectOn;
        public final boolean ball1RecoveryEjectOn;
        public final boolean ball2FeedEjectOn;
        public final boolean ball2RecoveryEjectOn;
        public final boolean ball3FeedEjectOn;
        public final boolean finishEjectOn;

        public final double ball1Start;
        public final double ball1End;
        public final double ball2Start;
        public final double ball2End;
        public final double ball3Start;
        public final double ball3End;

        public TimingConfig(double spinupTime,
                            double ball1FeedTime, double ball1RecoveryTime,
                            double ball2FeedTime, double ball2RecoveryTime,
                            double ball3FeedTime,
                            boolean spinupIntakeOn, boolean ball1FeedIntakeOn, boolean ball1RecoveryIntakeOn,
                            boolean ball2FeedIntakeOn, boolean ball2RecoveryIntakeOn, boolean ball3FeedIntakeOn,
                            boolean finishIntakeOn,
                            boolean spinupEjectOn, boolean ball1FeedEjectOn, boolean ball1RecoveryEjectOn,
                            boolean ball2FeedEjectOn, boolean ball2RecoveryEjectOn, boolean ball3FeedEjectOn,
                            boolean finishEjectOn) {
            this.spinupTime = spinupTime;
            this.ball1FeedTime = ball1FeedTime;
            this.ball1RecoveryTime = ball1RecoveryTime;
            this.ball2FeedTime = ball2FeedTime;
            this.ball2RecoveryTime = ball2RecoveryTime;
            this.ball3FeedTime = ball3FeedTime;

            this.spinupIntakeOn = spinupIntakeOn;
            this.ball1FeedIntakeOn = ball1FeedIntakeOn;
            this.ball1RecoveryIntakeOn = ball1RecoveryIntakeOn;
            this.ball2FeedIntakeOn = ball2FeedIntakeOn;
            this.ball2RecoveryIntakeOn = ball2RecoveryIntakeOn;
            this.ball3FeedIntakeOn = ball3FeedIntakeOn;
            this.finishIntakeOn = finishIntakeOn;

            this.spinupEjectOn = spinupEjectOn;
            this.ball1FeedEjectOn = ball1FeedEjectOn;
            this.ball1RecoveryEjectOn = ball1RecoveryEjectOn;
            this.ball2FeedEjectOn = ball2FeedEjectOn;
            this.ball2RecoveryEjectOn = ball2RecoveryEjectOn;
            this.ball3FeedEjectOn = ball3FeedEjectOn;
            this.finishEjectOn = finishEjectOn;

            this.ball1Start = spinupTime;
            this.ball1End = ball1Start + ball1FeedTime;
            this.ball2Start = ball1End + ball1RecoveryTime;
            this.ball2End = ball2Start + ball2FeedTime;
            this.ball3Start = ball2End + ball2RecoveryTime;
            this.ball3End = ball3Start + ball3FeedTime;
        }
    }

    private static final TimingConfig STANDARD_TIMING = new TimingConfig(
            3.0, 0.15, 1.5, 0.15, 1.5, 0.15,
            false, true, true, true, true, true, false,
            false, false, false, false, false, false, false
    );

    private static final TimingConfig FAST_RECOVERY_TIMING = new TimingConfig(
            3.0, 0.15, 0.5, 0.3, 0.5, 0.3,
            false, true, true, true, true, true, false,
            false, false, false, false, false, false, false
    );

    public static Configuration getConfiguration(AutonPath path, ShootingPosition position) {
        switch (path) {
            case BLUE_FAR:
                return getBlueFarConfiguration(position);
            case BLUE_NEAR:
                return getBlueNearConfiguration(position);
            case RED_FAR:
                return getRedFarConfiguration(position);
            case RED_NEAR:
                return getRedNearConfiguration(position);
            default:
                throw new IllegalArgumentException("Unknown autonomous path: " + path);
        }
    }

    private static Configuration getBlueFarConfiguration(ShootingPosition position) {
        return new Configuration(1.0, 0.6, 0.6, 6.0, STANDARD_TIMING);
    }

    private static Configuration getBlueNearConfiguration(ShootingPosition position) {
        switch (position) {
            case PRELOAD:
                return new Configuration(0.5, 0.33, 1.0, 6.0, STANDARD_TIMING);
            case SET_1:
                return new Configuration(0., 0.33, 1.0, 6.0, STANDARD_TIMING);
            case SET_2:
                return new Configuration(0.5, 0.33, 1.0, 6.0, STANDARD_TIMING);
            default:
                throw new IllegalArgumentException("Unknown shooting position: " + position);
        }
    }

    private static Configuration getRedFarConfiguration(ShootingPosition position) {
        return new Configuration(1.0, 0.6, 0.3, 6.0, FAST_RECOVERY_TIMING);
    }

    private static Configuration getRedNearConfiguration(ShootingPosition position) {
        return new Configuration(0.75, 0.55, 0.6, 6.0, FAST_RECOVERY_TIMING);
    }

    public static void performShooting(Launcher launcher, IntakeTransfer intakeTransfer,
                                       Timer shootTimer, Configuration config) {
        if (launcher.flyWheelMotor != null) {
            launcher.flyWheelMotor.setPower(config.flywheelPower);
        }
        if (launcher.flyWheelMotor2 != null) {
            launcher.flyWheelMotor2.setPower(config.flywheelPower);
        }
        launcher.setHoodPosition(config.hoodPosition);
        launcher.setSpinning(true);

        double elapsed = shootTimer.getElapsedTimeSeconds();
        TimingConfig timing = config.timing;

        if (elapsed < timing.spinupTime) {
            intakeTransfer.transferDown();
            controlIntakeEject(intakeTransfer, timing.spinupIntakeOn, timing.spinupEjectOn);
        }
        else if (elapsed >= timing.ball1Start && elapsed < timing.ball1End) {
            intakeTransfer.transferUp();
            controlIntakeEject(intakeTransfer, timing.ball1FeedIntakeOn, timing.ball1FeedEjectOn);
        }
        else if (elapsed >= timing.ball1End && elapsed < timing.ball2Start) {
            intakeTransfer.transferDown();
            controlIntakeEject(intakeTransfer, timing.ball1RecoveryIntakeOn, timing.ball1RecoveryEjectOn);
        }
        else if (elapsed >= timing.ball2Start && elapsed < timing.ball2End) {
            intakeTransfer.transferUp();
            controlIntakeEject(intakeTransfer, timing.ball2FeedIntakeOn, timing.ball2FeedEjectOn);
        }
        else if (elapsed >= timing.ball2End && elapsed < timing.ball3Start) {
            intakeTransfer.transferDown();
            controlIntakeEject(intakeTransfer, timing.ball2RecoveryIntakeOn, timing.ball2RecoveryEjectOn);
        }
        else if (elapsed >= timing.ball3Start && elapsed < timing.ball3End) {
            intakeTransfer.transferUp();
            controlIntakeEject(intakeTransfer, timing.ball3FeedIntakeOn, timing.ball3FeedEjectOn);
        }
        else if (elapsed >= timing.ball3End) {
            intakeTransfer.transferDown();
            controlIntakeEject(intakeTransfer, timing.finishIntakeOn, timing.finishEjectOn);
        }
    }

    private static void controlIntakeEject(IntakeTransfer intakeTransfer,
                                           boolean intakeOn, boolean ejectOn) {
        if (ejectOn) {
            intakeTransfer.startEject(1.0);
        } else if (intakeOn) {
            intakeTransfer.startIntake();
        } else {
            intakeTransfer.stopIntake();
        }
    }

    public static String getCurrentPhase(Timer shootTimer, Configuration config) {
        double elapsed = shootTimer.getElapsedTimeSeconds();
        TimingConfig timing = config.timing;

        if (elapsed < timing.spinupTime) {
            return "SPINUP";
        } else if (elapsed < timing.ball1End) {
            return "BALL 1 FEED";
        } else if (elapsed < timing.ball2Start) {
            return "BALL 1 RECOVERY";
        } else if (elapsed < timing.ball2End) {
            return "BALL 2 FEED";
        } else if (elapsed < timing.ball3Start) {
            return "BALL 2 RECOVERY";
        } else if (elapsed < timing.ball3End) {
            return "BALL 3 FEED";
        } else {
            return "FINISH";
        }
    }
}