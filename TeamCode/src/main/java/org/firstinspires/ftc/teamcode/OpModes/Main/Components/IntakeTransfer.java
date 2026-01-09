package org.firstinspires.ftc.teamcode.OpModes.Main.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeTransfer {

    // Hardware names
    public static final String INTAKE_MOTOR_NAME = "intakeMotor";
    public static final String TRANSFER_SERVO_NAME = "transferServo";

    // Transfer servo positions
    public static final double TRANSFER_UP_POSITION = 0.7;
    public static final double TRANSFER_DOWN_POSITION = 0.75;

    // Motor settings
    public static final double INTAKE_POWER = 1.0;
    public static final double EJECT_POWER = -0.7;

    // Hardware
    private DcMotor intakeMotor;
    private Servo transferServo;
    private Telemetry telemetry;

    // State
    private boolean transferUp = false;
    private double currentIntakePower = 0.0;

    // Edge detection
    private boolean prevB = false;
    private boolean prevX = false;

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        intakeMotor = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0);

        transferServo = hardwareMap.get(Servo.class, TRANSFER_SERVO_NAME);
        transferServo.setPosition(TRANSFER_DOWN_POSITION);

        telemetry.addLine("IntakeTransfer Initialized (No Pivot Servo)");
    }

    public void update(Gamepad gamepad) {
        // Intake motor control
        if (gamepad.right_trigger > 0.05) {
            currentIntakePower = gamepad.right_trigger * EJECT_POWER;
        } else if (gamepad.left_trigger > 0.05) {
            currentIntakePower = gamepad.left_trigger * INTAKE_POWER;
        } else {
            currentIntakePower = 0;
        }
        intakeMotor.setPower(currentIntakePower);

        // Transfer ramp toggle
        if (gamepad.b && !prevB) {
            transferUp = true;
        }
        if (gamepad.x && !prevX) {
            transferUp = false;
        }

// Always command servo based on state
        transferServo.setPosition(
                transferUp ? TRANSFER_UP_POSITION : TRANSFER_DOWN_POSITION
        );

        prevB = gamepad.b;
        prevX = gamepad.x;

        // Telemetry
        telemetry.addData("Intake Motor Power", "%.2f", currentIntakePower);
        telemetry.addData("Transfer Ramp", transferUp ? "UP" : "DOWN");
    }
}
