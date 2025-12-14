package org.firstinspires.ftc.teamcode.OpModes.Main.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.HardwareConfig;

public class Flywheel {
    private Telemetry telemetry;
    public DcMotor flyWheelMotor = null;
    public DcMotor flyWheelMotor2 = null;
    private double flywheelPower = 0.8; // starting power
    private boolean spinning = false;   // flywheel state

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        flyWheelMotor = hardwareMap.get(DcMotor.class, HardwareConfig.FLYWHEEL_MOTOR);

        telemetry.addData("Status", "Initialized. Flywheel power: " + flywheelPower);
        telemetry.update();
    }

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, double initialPower) {
        this.flywheelPower = Math.max(0.0, Math.min(1.0, initialPower));
        initialize(hardwareMap, telemetry);
    }

    public void update() {
        flyWheelMotor.setPower(spinning ? flywheelPower : 0);
        flyWheelMotor2.setPower(spinning ? flywheelPower : 0);
    }

    public void setSpinning(boolean spinning) {
        this.spinning = spinning;
    }

    public boolean isSpinning() {
        return spinning;
    }

    public void setPower(double power) {
        this.flywheelPower = Math.max(0.0, Math.min(1.0, power));
    }

    public double getPower() {
        return flywheelPower;
    }

    public void adjustPower(double increment) {
        flywheelPower = Math.max(0.0, Math.min(1.0, flywheelPower + increment));
    }
}

