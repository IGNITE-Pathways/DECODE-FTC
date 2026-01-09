package org.firstinspires.ftc.teamcode.OpModes.IndividualTest.OldTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.OpModes.Main.Components.OldComponents.Lift;

@TeleOp(name="Test: Parking", group="Test")
public class ParkingTest extends OpMode {

    private Lift lift;

    @Override
    public void init() {
        lift = new Lift();
        lift.initialize(hardwareMap, telemetry);
        telemetry.addLine("Lift Test Initialized");
        telemetry.addLine("D-Pad Up = go up, D-Pad Down = go down (click once)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update lift with gamepad inputs
        lift.update(gamepad1.dpad_up, gamepad1.dpad_down);

        // Telemetry
        telemetry.addData("Lift State", lift.getState());
        telemetry.addData("Left Viper Pos", lift.getLeftPosition());
        telemetry.addData("Right Viper Pos", lift.getRightPosition());
        telemetry.update();
    }
}
