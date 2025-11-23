package OpModes.IndividualTest;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Axon Micro CR: Button Direction", group = "ProgrammingBoardShooter")
public class BallLauncherServo extends LinearOpMode {

    private CRServo BallLauncher;

    // Tweakables
    private double speed = 0.60;            // default spin speed (0..1)
    private static final double STEP = 0.05; // d-pad speed step
    private static final double STOP_TRIM = 0.00; // small offset if servo creeps; e.g., -0.02..+0.02

    private enum Direction { FORWARD, REVERSE, STOP }
    private Direction lastDir = Direction.FORWARD; // remembered for toggle



    // Y → toggle direction

    // A → stop

    // RB (right bumper) → spin forward

    // LB (left bumper) → spin reverse



    @Override
    public void runOpMode() throws InterruptedException {
        // IMPORTANT: In the RC config, set "hoodservo" as CRServo on SERVO Port 1
        BallLauncher = hardwareMap.get(CRServo.class, "balllauncherservo");

        telemetry.addLine("Axon CR ready. RB=FWD, LB=REV, A=STOP, Y=TOGGLE, D-pad=Speed");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Direction buttons
            if (gamepad1.right_bumper) {
                lastDir = Direction.FORWARD;
                BallLauncher.setPower(+speed);
            } else if (gamepad1.left_bumper) {
                lastDir = Direction.REVERSE;
                BallLauncher.setPower(-speed);
            }

            // Stop
            if (gamepad1.a) {
                lastDir = Direction.STOP;
                BallLauncher.setPower(0.0 + STOP_TRIM);
            }

            // Toggle last non-stop direction
            if (gamepad1.y) {
                if (lastDir == Direction.FORWARD) {
                    lastDir = Direction.REVERSE;
                    BallLauncher.setPower(-speed);
                } else if (lastDir == Direction.REVERSE) {
                    lastDir = Direction.FORWARD;
                    BallLauncher.setPower(+speed);
                } else { // was STOP -> go forward
                    lastDir = Direction.FORWARD;
                    BallLauncher.setPower(+speed);
                }
                // simple debounce
                sleep(150);
            }

            // Speed adjust
            if (gamepad1.dpad_up) {
                speed = Math.min(1.0, speed + STEP);
                applyIfSpinning();
                sleep(120);
            } else if (gamepad1.dpad_down) {
                speed = Math.max(0.10, speed - STEP); // keep a sensible minimum
                applyIfSpinning();
                sleep(120);
            }

            telemetry.addData("Speed", "%.2f", speed);
            telemetry.addData("State", lastDir);
            telemetry.addLine("RB=FWD  LB=REV  A=STOP  Y=TOGGLE  D-pad±=speed");
            telemetry.update();

            sleep(20);
        }
    }

    // Keep direction but update power when speed changes
    private void applyIfSpinning() {
        switch (lastDir) {
            case FORWARD:
                BallLauncher.setPower(+speed);
                break;
            case REVERSE:
                BallLauncher.setPower(-speed);
                break;
            case STOP:
                BallLauncher.setPower(0.0 + STOP_TRIM);
                break;
        }
    }
}