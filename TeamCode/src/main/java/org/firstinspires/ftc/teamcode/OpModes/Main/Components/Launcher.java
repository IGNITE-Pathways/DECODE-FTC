package org.firstinspires.ftc.teamcode.OpModes.Main.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
// import org.firstinspires.ftc.teamcode.ProgrammingBoard.ProgrammingBoardOTHER;
import org.firstinspires.ftc.teamcode.Constants.HardwareConfig;

public class Launcher {
    // private ProgrammingBoardOTHER board;
    private Servo hoodServo;
    private Telemetry telemetry;
    public DcMotor flyWheelMotor = null;
    public DcMotor flyWheelMotor2 = null;

    private double flywheelPower = 0.75; // starting power
    private boolean spinning = false;   // flywheel state
    private double hoodPosition = 0.8;  // hood servo position

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        // board = new ProgrammingBoardOTHER();
        // board.initializeComponents(hardwareMap);
        
        hoodServo = hardwareMap.get(Servo.class, HardwareConfig.HOOD_SERVO);
        if (hoodServo != null) {
            hoodServo.setPosition(hoodPosition);
        }
        
        flyWheelMotor = hardwareMap.get(DcMotor.class, HardwareConfig.FLYWHEEL_MOTOR);
        flyWheelMotor2 = hardwareMap.get(DcMotor.class, HardwareConfig.FLYWHEEL_MOTOR_2);
        
        telemetry.addData("Status", "Initialized. Flywheel power: " + flywheelPower);
        telemetry.update();
    }

    public void update() {
        flyWheelMotor.setPower(spinning ? flywheelPower : 0);
        flyWheelMotor2.setPower(spinning ? flywheelPower : 0);
    }

    // Flywheel methods
    public void setSpinning(boolean spinning) {
        this.spinning = spinning;
    }

    public boolean isSpinning() {
        return spinning;
    }

    public void setPower(double power) {
        this.flywheelPower = power;
    }

    public double getPower() {
        return flywheelPower;
    }

    // Hood methods
    public void setHoodPosition(double position) {
        hoodPosition = Math.max(0.0, Math.min(1.0, position));
        if (hoodServo != null) {
            hoodServo.setPosition(hoodPosition);
        }
    }

    public double getHoodPosition() {
        return hoodPosition;
    }

    public void adjustHoodPosition(double increment) {
        setHoodPosition(hoodPosition + increment);
    }

    public void incrementHood() {
        adjustHoodPosition(0.05);
    }

    public void decrementHood() {
        adjustHoodPosition(-0.05);
    }
}

