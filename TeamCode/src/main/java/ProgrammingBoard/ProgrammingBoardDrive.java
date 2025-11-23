package ProgrammingBoard;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ProgrammingBoardDrive {

    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;

    public void initializeComponents(HardwareMap hwMap) {
        leftFrontDrive = hwMap.get(DcMotor.class, "leftfrontmotor");
        leftBackDrive = hwMap.get(DcMotor.class, "leftbackmotor");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightfrontmotor");
        rightBackDrive = hwMap.get(DcMotor.class, "rightbackmotor");

        // ✅ This is the correct FTC standard mecanum setup
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }
}
