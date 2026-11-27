package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Flywheel PIDF Tuner", group = "Tuning")
public class FlywheelPIDFTuner extends LinearOpMode {

    private DcMotorEx shooter;

    // Original PIDF values (used for reset)
    private static final double ORIG_P = 80.8;
    private static final double ORIG_I = 0.1196;
    private static final double ORIG_D = 0.0;
    private static final double ORIG_F = 11.9588;

    // Current PIDF values
    private double p = ORIG_P;
    private double i = ORIG_I;
    private double d = ORIG_D;
    private double f = ORIG_F;

    // Flywheel target
    private double targetVelocity = 1550;

    // PIDF selection
    private int selected = 0;

    // Adjustment multipliers
    private final double[] multipliers = {
            1.0, 0.1, 0.01, 0.001
    };

    private int multiplierIndex = 0;

    private boolean running = false;

    // Spin-up timing
    private final double VELOCITY_TOLERANCE = 30;   // ticks/sec, adjust to taste
    private double spinUpStartTime = 0;
    private double spinUpTime = -1;                 // -1 = not reached yet
    private boolean reachedTarget = false;

    // Button states
    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastLeft = false;
    private boolean lastRight = false;
    private boolean lastR1 = false;
    private boolean lastCross = false;
    private boolean lastTriangle = false;
    private boolean lastSquare = false;
    private boolean lastCircle = false;

    @Override
    public void runOpMode() {

        shooter = hardwareMap.get(
                DcMotorEx.class, "shooter"
        );

        shooter.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        shooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        applyPIDF();

        telemetry.addLine("FLYWHEEL PIDF TUNER");
        telemetry.addLine("FTC SDK 12.0.0");
        telemetry.addLine("Press TRIANGLE to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // SELECT PIDF
            if (gamepad1.dpad_right && !lastRight) {
                selected = (selected + 1) % 4;
            }
            lastRight = gamepad1.dpad_right;

            if (gamepad1.dpad_left && !lastLeft) {
                selected = (selected + 3) % 4;
            }
            lastLeft = gamepad1.dpad_left;

            // CYCLE MULTIPLIER
            if (gamepad1.right_bumper && !lastR1) {
                multiplierIndex =
                        (multiplierIndex + 1) % multipliers.length;
            }
            lastR1 = gamepad1.right_bumper;

            double step = multipliers[multiplierIndex];

            // INCREASE
            if (gamepad1.dpad_up && !lastUp) {
                changePIDF(step);
            }
            lastUp = gamepad1.dpad_up;

            // DECREASE
            if (gamepad1.dpad_down && !lastDown) {
                changePIDF(-step);
            }
            lastDown = gamepad1.dpad_down;

            // APPLY PIDF
            if (gamepad1.cross && !lastCross) {
                applyPIDF();
            }
            lastCross = gamepad1.cross;

            // START / STOP
            if (gamepad1.triangle && !lastTriangle) {
                running = !running;

                if (running) {
                    // Start timing
                    spinUpStartTime = getRuntime();
                    spinUpTime = -1;
                    reachedTarget = false;
                }
            }
            lastTriangle = gamepad1.triangle;

            // RESET PIDF
            if (gamepad1.square && !lastSquare) {

                p = ORIG_P;
                i = ORIG_I;
                d = ORIG_D;
                f = ORIG_F;

                applyPIDF();
            }
            lastSquare = gamepad1.square;

            // EMERGENCY STOP
            if (gamepad1.circle && !lastCircle) {
                running = false;
                reachedTarget = false;
            }
            lastCircle = gamepad1.circle;

            // FLYWHEEL CONTROL
            if (running) {
                shooter.setVelocity(targetVelocity);
            } else {
                shooter.setVelocity(0);
            }

            double actualVelocity = shooter.getVelocity();

            // SPIN-UP TIME DETECTION
            if (running && !reachedTarget
                    && Math.abs(targetVelocity - actualVelocity) <= VELOCITY_TOLERANCE) {
                spinUpTime = getRuntime() - spinUpStartTime;
                reachedTarget = true;
            }

            // Spin-up display text
            String spinText;
            if (spinUpTime >= 0) {
                spinText = String.format("%.2f s", spinUpTime);
            } else if (running) {
                spinText = String.format("%.2f s (spinning up...)",
                        getRuntime() - spinUpStartTime);
            } else {
                spinText = "--";
            }

            // TELEMETRY
            telemetry.addLine("===== PIDF TUNER =====");

            telemetry.addData("Selected",
                    selected == 0 ? "P" :
                            selected == 1 ? "I" :
                                    selected == 2 ? "D" : "F");

            telemetry.addLine("----------------------");

            telemetry.addData("P", "%.4f", p);
            telemetry.addData("I", "%.4f", i);
            telemetry.addData("D", "%.4f", d);
            telemetry.addData("F", "%.4f", f);

            telemetry.addLine("----------------------");

            telemetry.addData("Multiplier",
                    "%.3f", step);

            telemetry.addData("Target Velocity",
                    "%.0f", targetVelocity);

            telemetry.addData("Actual Velocity",
                    "%.0f", actualVelocity);

            telemetry.addData("Velocity Error",
                    "%.0f", targetVelocity - actualVelocity);

            telemetry.addData("Spin-Up Time", spinText);

            telemetry.addData("Status",
                    running ? "RUNNING" : "STOPPED");

            telemetry.addData("PIDF",
                    "Press X to apply");

            telemetry.update();
        }

        shooter.setVelocity(0);
    }

    // CHANGE SELECTED PIDF VALUE
    private void changePIDF(double amount) {

        switch (selected) {

            case 0:
                p = Math.max(0, p + amount);
                break;

            case 1:
                i = Math.max(0, i + amount);
                break;

            case 2:
                d = Math.max(0, d + amount);
                break;

            case 3:
                f = Math.max(0, f + amount);
                break;
        }
    }

    // APPLY PIDF TO MOTOR
    private void applyPIDF() {

        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(p, i, d, f)
        );
    }
}