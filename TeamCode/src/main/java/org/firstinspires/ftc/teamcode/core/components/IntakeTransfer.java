package org.firstinspires.ftc.teamcode.core.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.constants.HardwareConfig;
import org.firstinspires.ftc.teamcode.core.constants.ShooterConstants;

public class IntakeTransfer {

    // Motor settings
    public static final double EJECT_POWER = ShooterConstants.INTAKE_DEFAULT_POWER;
    public static final double INTAKE_POWER = 1.0;

    // Hardware
    private DcMotor intakeMotor;
    private Servo transferServo;
    private Telemetry telemetry;

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
        // EJECT_POWER is negative, so this runs opposite to intake
        intakeMotor.setPower(Math.abs(power) * EJECT_POWER);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }

    public double getIntakePower() {
        return intakeMotor.getPower();
    }

    // ==================== TRANSFER SERVO CONTROL ====================

    public void transferUp() {
        if (transferServo != null) {
            transferServo.setPosition(ShooterConstants.TRANSFER_UP_POSITION);
        }
    }

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
}
