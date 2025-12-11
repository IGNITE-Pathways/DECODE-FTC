package org.firstinspires.ftc.teamcode.ProgrammingBoard;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Constants.HardwareConfig;

public class ProgrammingBoardOTHER {
    public DcMotor flyWheelMotor = null;
    public DcMotor flyWheelMotor2 = null;
    public Servo hoodServo;

    public Servo BallLauncherServo;

    public CRServo intakeServo;

    public Servo indexServo;

    public CRServo kickerWheel;

    public NormalizedColorSensor intakeColorSensor;

    public void initializeComponents(HardwareMap hwMap) {


        //indexServo = hwMap.get(Servo.class, HardwareConfig.INDEX_SERVO); // port 2

        //intakeServo = hwMap.get(CRServo.class, HardwareConfig.INTAKE_SERVO);  // port 0

        flyWheelMotor = hwMap.get(DcMotor.class, HardwareConfig.FLYWHEEL_MOTOR);

        flyWheelMotor2 = hwMap.get(DcMotor.class, HardwareConfig.FLYWHEEL_MOTOR_2);

//        flyWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //hoodServo = hwMap.get(Servo.class, HardwareConfig.HOOD_SERVO);

        //BallLauncherServo = hwMap.get(Servo.class, HardwareConfig.BALL_LAUNCHER);

        //kickerWheel = hwMap.get(CRServo.class, HardwareConfig.KICKER_WHEEL);


    }

}
