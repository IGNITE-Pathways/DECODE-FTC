package ProgrammingBoard;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

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

        intakeColorSensor = hwMap.get(NormalizedColorSensor.class, "intakeSensor");

        indexServo = hwMap.get(Servo.class, "indexServo"); // port 2

        intakeServo = hwMap.get(CRServo.class, "intakeservo");  // port 0

        flyWheelMotor = hwMap.get(DcMotor.class, "flywheelmotor");

        flyWheelMotor2 = hwMap.get(DcMotor.class, "flywheelmotor2");

//        flyWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hoodServo = hwMap.get(Servo.class, "hoodservo");

        BallLauncherServo = hwMap.get(Servo.class, "balllauncher");

        kickerWheel = hwMap.get(CRServo.class, "kickerwheel");


    }

}
