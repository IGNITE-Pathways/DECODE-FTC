package OpModes.InProgress;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import ProgrammingBoard.ProgrammingBoardOTHER;
@Disabled
@TeleOp(name = "Main Op Mode", group = "Linear OpMode")
public class IndexingOrShootSystem extends LinearOpMode {
    ProgrammingBoardOTHER board = new ProgrammingBoardOTHER();
    private NormalizedColorSensor test_color;

    int flag = 0;
    boolean imperfect = false;
    String[] need_colors = {"purple", "purple", "green"}; // pull pattern

    @Override
    public void runOpMode() {
        test_color = hardwareMap.get(NormalizedColorSensor.class, "shooterSensor");
        board.initializeComponents(hardwareMap);
        boolean isPurple = false;
        boolean isGreen = false;
        waitForStart();
        while (opModeIsActive()) {
            NormalizedRGBA colors = test_color.getNormalizedColors();
            float hue = JavaUtil.colorToHue(colors.toColor());


            if (hue < 100) {
                isPurple = false;
                isGreen = false;
            }
            else if(hue < 350 && hue> 160){
                isPurple = true;
            }
            else {
                isGreen = true;
            }
            String neededBall = need_colors[flag];
            if (gamepad1.square || imperfect){
                if ((isPurple && neededBall.equals("purple")) || (isGreen && neededBall.equals("green"))) {
                    //insert flywheel shoot code
                    board.flyWheelMotor.setPower(1);
                    sleep(3000);
                    board.flyWheelMotor.setPower(0);
                    //f
                    flag += 1;
                    if (flag > 2){
                        flag = 0;
                    }
                    imperfect = false;
                }
                else{
                    //set hood positition low and shoot so it lands in intake
                    board.flyWheelMotor.setPower(0.6);
                    sleep(3000);
                    board.flyWheelMotor.setPower(0);

                    imperfect = true;
                }

            }
            telemetry.addData("Needed Ball", neededBall);
            telemetry.addData("IsPurple?", isPurple);
            telemetry.addData("IsGreen?", isGreen);
            telemetry.addData("Hue:", hue);
            telemetry.update();

        }

        }
    }


