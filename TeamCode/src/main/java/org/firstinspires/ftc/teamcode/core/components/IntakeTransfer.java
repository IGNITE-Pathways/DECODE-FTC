package org.firstinspires.ftc.teamcode.core.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;
import org.firstinspires.ftc.teamcode.core.constants.ShooterConstants;

public class IntakeTransfer {

    // Motor settings
    public static final double EJECT_POWER = ShooterConstants.INTAKE_DEFAULT_POWER;
    public static final double INTAKE_POWER = 1.0;

    // Eject-then-intake sequence timing
    private static final double EJECT_DURATION_MS = 300;  // How long to eject before switching to intake

    // Hardware
    private DcMotor intakeMotor;
    private Servo transferServo;
    private Telemetry telemetry;

    // Sequence state
    private boolean isSequenceActive = false;
    private ElapsedTime sequenceTimer = new ElapsedTime();

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        try {
            intakeMotor = hardwareMap.get(DcMotor.class, HardwareConfig.INTAKE_MOTOR);
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setPower(0);
            telemetry.addLine("Intake Motor: OK");
        } catch (Exception e) {
            telemetry.addLine("Intake Motor: NOT FOUND - " + HardwareConfig.INTAKE_MOTOR);
        }

        try {
            transferServo = hardwareMap.get(Servo.class, HardwareConfig.TRANSFER_SERVO);
            transferServo.setPosition(ShooterConstants.TRANSFER_DOWN_POSITION);
            telemetry.addLine("Transfer Servo: OK");
        } catch (Exception e) {
            telemetry.addLine("Transfer Servo: NOT FOUND - " + HardwareConfig.TRANSFER_SERVO);
            transferServo = null;
        }

        telemetry.addLine("IntakeTransfer Initialized");
    }

    // ==================== INTAKE MOTOR CONTROL ====================
    // These methods work for BOTH TeleOp and Auton

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    /**
     * Start intake at full power.
     * Works in both TeleOp and Auton.
     */
    public void startIntake() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    /**
     * Start intake with variable power (e.g., from trigger).
     * Works in both TeleOp and Auton.
     */
    public void startIntake(double power) {
        intakeMotor.setPower(Math.abs(power) * INTAKE_POWER);
    }

    /**
     * Start ejecting at full power.
     * Works in both TeleOp and Auton.
     */
    public void startEject() {
        intakeMotor.setPower(EJECT_POWER);
    }

    /**
     * Start ejecting with variable power (e.g., from trigger).
     * Works in both TeleOp and Auton.
     */
    public void startEject(double power) {
        // EJECT_POWER is negative, so this runs opposite to intake
        intakeMotor.setPower(Math.abs(power) * EJECT_POWER);
    }

    /**
     * Stop the intake motor.
     * Works in both TeleOp and Auton.
     */
    public void stopIntake() {
        intakeMotor.setPower(0);
    }

    public double getIntakePower() {
        return intakeMotor.getPower();
    }

    // ==================== TRANSFER SERVO CONTROL ====================
    // These methods work for BOTH TeleOp and Auton

    /**
     * Move transfer servo to UP position.
     * Works in both TeleOp and Auton.
     */
    public void transferUp() {
        if (transferServo != null) {
            transferServo.setPosition(ShooterConstants.TRANSFER_UP_POSITION);
        }
    }

    /**
     * Move transfer servo to DOWN position.
     * Works in both TeleOp and Auton.
     */
    public void transferDown() {
        if (transferServo != null) {
            transferServo.setPosition(ShooterConstants.TRANSFER_DOWN_POSITION);
        }
    }

    public void setTransferPosition(double position) {
        if (transferServo != null) {
            transferServo.setPosition(position);
        }
    }

    public double getTransferPosition() {
        if (transferServo != null) {
            return transferServo.getPosition();
        }
        return 0;
    }

    public boolean isTransferUp() {
        if (transferServo != null) {
            return Math.abs(transferServo.getPosition() - ShooterConstants.TRANSFER_UP_POSITION) < 0.05;
        }
        return false;
    }

    // ==================== EJECT-THEN-INTAKE SEQUENCE ====================

    // === FOR AUTON (BLOCKING) - Simple to use ===

    /**
     * AUTON METHOD - Ejects for 300ms, then switches to intake.
     * This method BLOCKS and waits - perfect for autonomous!
     *
     * Example in auton:
     *   intakeTransfer.ejectThenIntakeBlocking();
     *   // Code here runs AFTER sequence completes
     */
    public void ejectThenIntakeBlocking() {
        intakeMotor.setPower(EJECT_POWER);
        sleep((long) EJECT_DURATION_MS);
        intakeMotor.setPower(INTAKE_POWER);
    }

    // === FOR TELEOP (NON-BLOCKING) - Needs update() in loop ===

    /**
     * TELEOP METHOD - Start the eject-then-intake sequence.
     * Must call updateSequence() in your main loop!
     *
     * Example in teleop:
     *   if (gamepad1.x) {
     *       intakeTransfer.startEjectThenIntake();
     *   }
     *   intakeTransfer.updateSequence(); // Add this in your main loop
     */
    public void startEjectThenIntake() {
        isSequenceActive = true;
        sequenceTimer.reset();
        intakeMotor.setPower(EJECT_POWER);
    }

    /**
     * TELEOP METHOD - Update the sequence. Call this in your main loop!
     * @return true if sequence is still running, false if complete
     */
    public boolean updateSequence() {
        if (!isSequenceActive) {
            return false;
        }

        if (sequenceTimer.milliseconds() >= EJECT_DURATION_MS) {
            // Switch to intake
            intakeMotor.setPower(INTAKE_POWER);
            isSequenceActive = false;
            return false;
        }

        return true;
    }

    /**
     * Check if the eject-then-intake sequence is currently running.
     */
    public boolean isSequenceActive() {
        return isSequenceActive;
    }

    /**
     * Cancel the eject-then-intake sequence and stop the motor.
     */
    public void cancelSequence() {
        isSequenceActive = false;
        stopIntake();
    }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
