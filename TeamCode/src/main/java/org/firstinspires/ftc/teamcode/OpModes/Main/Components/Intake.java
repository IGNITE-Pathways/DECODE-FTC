package org.firstinspires.ftc.teamcode.OpModes.Main.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.HardwareConfig;

public class Intake {
    private CRServo intakeServo;
    private Telemetry telemetry;
    
    private double power = 0.0;
    private boolean running = false;

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeServo = hardwareMap.get(CRServo.class, HardwareConfig.INTAKE_SERVO);
        
        if (intakeServo != null) {
            intakeServo.setPower(0.0);
        }
        
        telemetry.addLine("Intake Component Initialized");
        telemetry.update();
    }

    public void setPower(double power) {
        this.power = Math.max(-1.0, Math.min(1.0, power));
        if (intakeServo != null) {
            intakeServo.setPower(this.power);
        }
        running = Math.abs(this.power) > 0.01;
    }

    public void start() {
        setPower(0.6); // Default intake power
    }

    public void start(double power) {
        setPower(power);
    }

    public void stop() {
        setPower(0.0);
    }

    public void reverse() {
        setPower(-0.6); // Reverse at default power
    }

    public double getPower() {
        return power;
    }

    public boolean isRunning() {
        return running;
    }

    public void update() {
        // Update telemetry if needed
        telemetry.addData("Intake Power", "%.2f", power);
        telemetry.addData("Intake Status", running ? "Running" : "Stopped");
    }
}
