package org.firstinspires.ftc.teamcode.OpModes.Main.Components.OldComponents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.HardwareConfig;

public class Lift {
    // Hardware
    private DcMotor leftViper;
    private DcMotor rightViper;
    private Telemetry telemetry;

    // Constants
    private static final int TOP_POSITION = 12373;  // top encoder ticks
    private static final int BOTTOM_POSITION = 0;   // bottom encoder ticks
    private static final double UP_POWER = 0.8;     // fast up
    private static final double DOWN_POWER = 0.3;   // slower down
    private static final double HOLD_POWER = 0.2;   // hold power after stopping

    // State management
    public enum State { IDLE, MOVING_UP, MOVING_DOWN, HOLDING }
    private State state = State.IDLE;

    // To detect button clicks
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        
        // Get motors from hardware map
        leftViper = hardwareMap.get(DcMotor.class, HardwareConfig.LEFT_VIPER);
        rightViper = hardwareMap.get(DcMotor.class, HardwareConfig.RIGHT_VIPER);

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

        telemetry.addLine("Lift Component Initialized");
        telemetry.addLine("D-Pad Up = go up, D-Pad Down = go down (click once)");
    }

    public void moveUp() {
        state = State.MOVING_UP;
        leftViper.setTargetPosition(TOP_POSITION);
        rightViper.setTargetPosition(TOP_POSITION);
        leftViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftViper.setPower(UP_POWER);
        rightViper.setPower(UP_POWER);
    }

    public void moveDown() {
        state = State.MOVING_DOWN;
        leftViper.setTargetPosition(BOTTOM_POSITION);
        rightViper.setTargetPosition(BOTTOM_POSITION);
        leftViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftViper.setPower(DOWN_POWER);
        rightViper.setPower(DOWN_POWER);
    }

    public void update(boolean dpadUp, boolean dpadDown) {
        // D-Pad Up clicked
        if (dpadUp && !dpadUpPrev) {
            moveUp();
        }

        // D-Pad Down clicked
        if (dpadDown && !dpadDownPrev) {
            moveDown();
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
        telemetry.addData("Lift State", state);
        telemetry.addData("Left Viper Pos", leftViper.getCurrentPosition());
        telemetry.addData("Right Viper Pos", rightViper.getCurrentPosition());
    }

    public State getState() {
        return state;
    }

    public int getLeftPosition() {
        return leftViper != null ? leftViper.getCurrentPosition() : 0;
    }

    public int getRightPosition() {
        return rightViper != null ? rightViper.getCurrentPosition() : 0;
    }
}
