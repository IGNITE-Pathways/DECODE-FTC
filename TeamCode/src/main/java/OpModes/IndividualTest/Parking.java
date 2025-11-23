package OpModes.IndividualTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Parking Individual Control", group="Individual Test")
public class Parking extends OpMode {

    private DcMotor leftViper;
    private DcMotor rightViper;

    private static final int TOP_POSITION = 12373;  // top encoder ticks
    private static final int BOTTOM_POSITION = 0;   // bottom encoder ticks

    private static final double UP_POWER = 0.8;     // fast up
    private static final double DOWN_POWER = 0.3;   // slower down
    private static final double HOLD_POWER = 0.2;   // hold power after stopping

    private enum State { IDLE, MOVING_UP, MOVING_DOWN, HOLDING }
    private State state = State.IDLE;

    // To detect button clicks
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;

    @Override
    public void init() {
        // Get motors from hardware map
        leftViper = hardwareMap.get(DcMotor.class, "leftviper");
        rightViper = hardwareMap.get(DcMotor.class, "rightviper");

        // Reverse one motor so they move in sync
        rightViper.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        leftViper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightViper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Run with encoders
        leftViper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightViper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply brake
        leftViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialized");
        telemetry.addLine("D-Pad Up = go up, D-Pad Down = go down (click once)");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;

        // D-Pad Up clicked
        if (dpadUp && !dpadUpPrev) {
            state = State.MOVING_UP;
            leftViper.setTargetPosition(TOP_POSITION);
            rightViper.setTargetPosition(TOP_POSITION);
            leftViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftViper.setPower(UP_POWER);
            rightViper.setPower(UP_POWER);
        }

        // D-Pad Down clicked
        if (dpadDown && !dpadDownPrev) {
            state = State.MOVING_DOWN;
            leftViper.setTargetPosition(BOTTOM_POSITION);
            rightViper.setTargetPosition(BOTTOM_POSITION);
            leftViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftViper.setPower(DOWN_POWER);
            rightViper.setPower(DOWN_POWER);
        }

        // Update previous button states
        dpadUpPrev = dpadUp;
        dpadDownPrev = dpadDown;

        // Check if movement finished
        if ((state == State.MOVING_UP || state == State.MOVING_DOWN) &&
                !leftViper.isBusy() && !rightViper.isBusy()) {

            state = State.HOLDING;
            int pos1 = leftViper.getCurrentPosition();
            int pos2 = rightViper.getCurrentPosition();
            leftViper.setTargetPosition(pos1);
            rightViper.setTargetPosition(pos2);
            leftViper.setPower(HOLD_POWER);
            rightViper.setPower(HOLD_POWER);
        }

        // Telemetry
        telemetry.addData("State", state);
        telemetry.addData("Left Viper Pos", leftViper.getCurrentPosition());
        telemetry.addData("Right Viper Pos", rightViper.getCurrentPosition());
        telemetry.update();
    }
}
