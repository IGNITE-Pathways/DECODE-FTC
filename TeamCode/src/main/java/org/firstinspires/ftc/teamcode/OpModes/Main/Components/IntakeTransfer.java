package org.firstinspires.ftc.teamcode.OpModes.Main.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.HardwareConfig;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

public class IntakeTransfer {

    // Motor settings
    public static final double INTAKE_POWER = ShooterConstants.INTAKE_DEFAULT_POWER;
    public static final double EJECT_POWER = -0.7;

    // Hardware
    private DcMotor intakeMotor;
    private Servo transferServo;
    private Telemetry telemetry;

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        intakeMotor = hardwareMap.get(DcMotor.class, HardwareConfig.INTAKE_MOTOR);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0);

        transferServo = hardwareMap.get(Servo.class, HardwareConfig.TRANSFER_SERVO);
        transferServo.setPosition(ShooterConstants.TRANSFER_DOWN_POSITION);

        telemetry.addLine("IntakeTransfer Initialized");
    }

    // ==================== INTAKE MOTOR CONTROL ====================

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    public void startIntake() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    public void startIntake(double power) {
        intakeMotor.setPower(Math.abs(power) * INTAKE_POWER);
    }

    public void startEject() {
        intakeMotor.setPower(EJECT_POWER);
    }

    public void startEject(double power) {
        intakeMotor.setPower(-Math.abs(power) * Math.abs(EJECT_POWER));
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }

    public double getIntakePower() {
        return intakeMotor.getPower();
    }

    // ==================== TRANSFER SERVO CONTROL ====================

    public void transferUp() {
        transferServo.setPosition(ShooterConstants.TRANSFER_UP_POSITION);
    }

    public void transferDown() {
        transferServo.setPosition(ShooterConstants.TRANSFER_DOWN_POSITION);
    }

    public void setTransferPosition(double position) {
        transferServo.setPosition(position);
    }

    public double getTransferPosition() {
        return transferServo.getPosition();
    }

    public boolean isTransferUp() {
        return Math.abs(transferServo.getPosition() - ShooterConstants.TRANSFER_UP_POSITION) < 0.05;
    }
}
