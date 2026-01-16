package org.firstinspires.ftc.teamcode.core.components;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.constants.AllianceColor;

import java.util.List;

/**
 * Simplified Robot class - manages all active components.
 *
 * ACTIVE COMPONENTS:
 * - DriveTrain: Mecanum drive with speed modes
 * - Turret: AprilTag tracking servo
 * - Launcher: Flywheel + hood
 * - IntakeTransfer: Intake motor + transfer servo
 */
public class Robot {
    // Components
    private DriveTrain driveTrain;
    private Turret turret;
    private Launcher launcher;
    private IntakeTransfer intakeTransfer;

    private Telemetry telemetry;

    // Bulk caching for faster sensor reads (~2.5x improvement)
    private List<LynxModule> allHubs;

    /**
     * Initialize all robot components
     */
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        initialize(hardwareMap, telemetry, opMode, null);
    }

    /**
     * Initialize all robot components with alliance color
     */
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode, AllianceColor allianceColor) {
        this.telemetry = telemetry;

        // Enable bulk caching for ~2.5x faster sensor reads
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize components
        driveTrain = new DriveTrain();
        driveTrain.initialize(hardwareMap, telemetry);

        turret = new Turret();
        turret.initialize(hardwareMap, telemetry, allianceColor);

        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);

        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);

        telemetry.addLine("Robot Initialized (Bulk Caching: ON)");
        telemetry.update();
    }

    // ==================== UPDATE METHODS ====================

    public void updateDriveTrain(double forward, double strafe, double rotate) {
        driveTrain.update(forward, strafe, rotate);
    }

    public boolean updateTurret() {
        return turret.update();
    }

    public void updateLauncher() {
        launcher.update();
    }

    // ==================== FLYWHEEL CONTROL ====================

    public void startFlywheel() {
        launcher.setSpinning(true);
        launcher.update();
    }

    public void startFlywheel(double power) {
        launcher.setPower(power);
        launcher.setSpinning(true);
        launcher.update();
    }

    public void stopFlywheel() {
        launcher.setSpinning(false);
        launcher.update();
    }

    public boolean isFlywheelSpinning() {
        return launcher.isSpinning();
    }

    public void setFlywheelPower(double power) {
        launcher.setPower(power);
    }

    public double getFlywheelPower() {
        return launcher.getPower();
    }

    // ==================== HOOD CONTROL ====================

    public void setHoodPosition(double position) {
        launcher.setHoodPosition(position);
    }

    public double getHoodPosition() {
        return launcher.getHoodPosition();
    }

    public void adjustHood(double increment) {
        launcher.adjustHoodPosition(increment);
    }

    // ==================== TURRET CONTROL ====================

    public boolean isTurretAligned() {
        return turret.isAligned();
    }

    public void lockTurret() {
        turret.lock();
    }

    public void unlockTurret() {
        turret.unlock();
    }

    public void setTurretPosition(double position) {
        turret.setPositionDirect(position);
    }

    public double getDistance() {
        return turret.getDistance();
    }

    public int detectObeliskAprilTag() {
        return turret.detectObeliskAprilTag();
    }

    // ==================== STOP ALL ====================

    public void stopAll() {
        stopFlywheel();
        intakeTransfer.stopIntake();
        driveTrain.stopMotors();
    }

    // ==================== COMPONENT GETTERS ====================

    public DriveTrain getDriveTrain() {
        return driveTrain;
    }

    public Turret getTurret() {
        return turret;
    }

    public Launcher getLauncher() {
        return launcher;
    }

    public IntakeTransfer getIntakeTransfer() {
        return intakeTransfer;
    }

    // ==================== TELEMETRY ====================

    public void addTelemetry() {
        telemetry.addLine("=== ROBOT STATUS ===");
        telemetry.addData("Flywheel", isFlywheelSpinning() ? "ON" : "OFF");
        telemetry.addData("Hood", "%.2f", getHoodPosition());
        telemetry.addData("Turret", isTurretAligned() ? "ALIGNED" : "TRACKING");

        double dist = getDistance();
        if (dist > 0) {
            telemetry.addData("Distance", "%.1f ft", dist);
        }
    }
}
