package org.firstinspires.ftc.teamcode.ProgrammingBoard;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Constants.HardwareConfig;

public class ProgrammingBoardShooter {
    public DcMotor flyWheelMotor = null;
    public DcMotor flyWheelMotor2 = null;
    public Servo hoodServo;

    public void initializeComponents(HardwareMap hwMap) {
        flyWheelMotor = hwMap.get(DcMotor.class, HardwareConfig.FLYWHEEL_MOTOR);
        flyWheelMotor2 = hwMap.get(DcMotor.class, HardwareConfig.FLYWHEEL_MOTOR_2);
//        flyWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hoodServo = hwMap.get(Servo.class, HardwareConfig.HOOD_SERVO);
    }

}