package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;

/**
 * Test OpMode for the IntakeTransfer mechanism.
 *
 * CONTROLS:
 * - LT (Left Trigger): Run intake motor (variable speed)
 * - RT (Right Trigger): Reverse intake / eject balls
 * - B: Move transfer ramp UP (transfer to shooter)
 * - X: Move transfer ramp DOWN (ready position)
 *
 * This is a standalone test - no other robot components required.
 */
@TeleOp(name = "Test: IntakeTransfer", group = "Test")
public class IntakeTransferTest extends LinearOpMode {

    private IntakeTransfer intakeTransfer;

    // State machine
    private enum IntakeState { IDLE, INTAKING, EJECTING }
    private enum TransferState { DOWN, UP }

    private IntakeState intakeState = IntakeState.IDLE;
    private TransferState transferState = TransferState.DOWN;

    // Edge detection
    private boolean prevB = false;
    private boolean prevX = false;

    @Override
    public void runOpMode() {
        // Initialize the component
        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);

        // Display controls on init
        telemetry.addLine("=== IntakeTransfer Test ===");
        telemetry.addLine("");
        telemetry.addLine("INTAKE MOTOR:");
        telemetry.addLine("  LT = Run intake");
        telemetry.addLine("  RT = Eject / reverse");
        telemetry.addLine("");
        telemetry.addLine("TRANSFER RAMP:");
        telemetry.addLine("  B = Ramp UP (transfer)");
        telemetry.addLine("  X = Ramp DOWN (ready)");
        telemetry.addLine("");
        telemetry.addLine("Press START to begin...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read inputs
            boolean leftTriggerPressed = gamepad1.left_trigger > 0.05;
            boolean rightTriggerPressed = gamepad1.right_trigger > 0.05;

            // ==================== INTAKE STATE MACHINE ====================
            switch (intakeState) {
                case IDLE:
                    intakeTransfer.stopIntake();
                    if (leftTriggerPressed) {
                        intakeState = IntakeState.INTAKING;
                    } else if (rightTriggerPressed) {
                        intakeState = IntakeState.EJECTING;
                    }
                    break;

                case INTAKING:
                    intakeTransfer.startIntake(gamepad1.left_trigger);
                    if (!leftTriggerPressed) {
                        intakeState = IntakeState.IDLE;
                    } else if (rightTriggerPressed) {
                        intakeState = IntakeState.EJECTING;
                    }
                    break;

                case EJECTING:
                    intakeTransfer.startEject(gamepad1.right_trigger);
                    if (!rightTriggerPressed) {
                        intakeState = IntakeState.IDLE;
                    } else if (leftTriggerPressed && !rightTriggerPressed) {
                        intakeState = IntakeState.INTAKING;
                    }
                    break;
            }

            // ==================== TRANSFER STATE MACHINE ====================
            // Only move servo when button is pressed (not continuously)
            if (gamepad1.b && !prevB) {
                transferState = TransferState.UP;
                intakeTransfer.transferUp();
            }
            if (gamepad1.x && !prevX) {
                transferState = TransferState.DOWN;
                intakeTransfer.transferDown();
            }

            // Update edge detection
            prevB = gamepad1.b;
            prevX = gamepad1.x;

            // Display status
            telemetry.addLine("=== IntakeTransfer Status ===");
            telemetry.addLine("");
            telemetry.addData("Intake State", intakeState.name());
            telemetry.addData("Transfer State", transferState.name());
            telemetry.addData("Motor Power", "%.2f", intakeTransfer.getIntakePower());
            telemetry.addData("Transfer Up", intakeTransfer.isTransferUp() ? "YES" : "NO");
            telemetry.addLine("");
            telemetry.addLine("Controls: LT=intake, RT=eject, B/X=ramp");
            telemetry.update();
        }
    }
}
