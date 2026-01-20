package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.components.IntakeTransfer;
import org.firstinspires.ftc.teamcode.core.components.Launcher;

/**
 * Single Ball Shoot Test
 *
 * Y = Start sequence (slow eject, then full intake to shoot)
 * RT = Manual intake
 */
@TeleOp(name = "Test: Single Ball Shoot", group = "Test")
public class SingleBallShootTest extends LinearOpMode {

    private IntakeTransfer intakeTransfer;
    private Launcher launcher;

    // Timing
    private static final double EJECT_TIME = 0.6;    // Slow eject duration
    private static final double INTAKE_TIME = 2.0;   // Full intake duration
    private static final double EJECT_POWER = -0.48;
    private static final double FLYWHEEL_POWER = 0.90;

    private ElapsedTime timer = new ElapsedTime();
    private boolean running = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() {
        intakeTransfer = new IntakeTransfer();
        intakeTransfer.initialize(hardwareMap, telemetry);
        launcher = new Launcher();
        launcher.initialize(hardwareMap, telemetry);

        telemetry.addLine("=== Single Ball Shoot Test ===");
        telemetry.addLine("Y = Auto sequence");
        telemetry.addLine("RT = Intake | LT = Eject");
        telemetry.addLine("Flywheel + Ramp: ALWAYS ON");
        telemetry.update();

        waitForStart();

        // Start flywheel and ramp immediately - keep running entire time
        launcher.setPower(FLYWHEEL_POWER);
        launcher.setSpinning(true);
        intakeTransfer.transferUp();

        while (opModeIsActive()) {
            // Y to start/stop sequence (flywheel and ramp always on)
            if (gamepad1.y && !lastY) {
                if (!running) {
                    // Start sequence
                    running = true;
                    timer.reset();
                } else {
                    // Cancel
                    running = false;
                    intakeTransfer.stopIntake();
                }
            }
            lastY = gamepad1.y;

            // Sequence logic
            if (running) {
                double t = timer.seconds();
                if (t < EJECT_TIME) {
                    // Phase 1: slow eject
                    intakeTransfer.setIntakePower(EJECT_POWER);
                } else if (t < EJECT_TIME + INTAKE_TIME) {
                    // Phase 2: full intake
                    intakeTransfer.startIntake(1.0);
                } else {
                    // Done (flywheel and ramp keep running)
                    running = false;
                    intakeTransfer.stopIntake();
                }
            } else {
                // Manual RT intake / LT eject when not running sequence
                if (gamepad1.right_trigger > 0.1) {
                    intakeTransfer.startIntake(gamepad1.right_trigger);
                } else if (gamepad1.left_trigger > 0.1) {
                    intakeTransfer.startEject(gamepad1.left_trigger);
                } else {
                    intakeTransfer.stopIntake();
                }
            }

            launcher.update();

            // Telemetry
            telemetry.addLine("=== Single Ball Shoot ===");
            telemetry.addData("Flywheel", "75% (always)");
            telemetry.addData("Ramp", "UP (always)");
            telemetry.addData("Sequence", running ? "RUNNING" : "IDLE");
            if (running) {
                double t = timer.seconds();
                telemetry.addData("Phase", t < EJECT_TIME ? "EJECT" : "INTAKE");
                telemetry.addData("Time", "%.1f s", t);
            }
            telemetry.addLine(running ? "Y=Cancel" : "Y=Start | RT=In | LT=Out");
            telemetry.update();
        }

        launcher.setSpinning(false);
        intakeTransfer.stopIntake();
    }
}
