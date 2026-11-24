//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//
//@TeleOp(name = "Flywheel PIDF Tuner", group = "Tuning")
//public class FlywheelPIDFTuner extends LinearOpMode {
//
//    private DcMotorEx shooter;
//
//    // Original PIDF values
//    private double p = 1.1959;
//    private double i = 0.1196;
//    private double d = 0.0;
//    private double f = 11.9588;
//
//    // Flywheel target
//    private double targetVelocity = 1550;
//
//    // PIDF selection
//    private int selected = 0;
//
//    // Adjustment multipliers
//    private final double[] multipliers = {
//            1.0, 0.1, 0.01, 0.001
//    };
//
//    private int multiplierIndex = 0;
//
//    private boolean running = false;
//
//    // Button states
//    private boolean lastUp = false;
//    private boolean lastDown = false;
//    private boolean lastLeft = false;
//    private boolean lastRight = false;
//    private boolean lastR1 = false;
//    private boolean lastCross = false;
//    private boolean lastTriangle = false;
//    private boolean lastSquare = false;
//    private boolean lastCircle = false;
//
//    @Override
//    public void runOpMode() {
//
//        shooter = hardwareMap.get(
//                DcMotorEx.class, "shooter"
//        );
//
//        shooter.setMode(
//                DcMotor.RunMode.RUN_USING_ENCODER
//        );
//
//        shooter.setZeroPowerBehavior(
//                DcMotor.ZeroPowerBehavior.FLOAT
//        );
//
//        applyPIDF();
//
//        telemetry.addLine("FLYWHEEL PIDF TUNER");
//        telemetry.addLine("FTC SDK 12.0.0");
//        telemetry.addLine("Press TRIANGLE to start");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            // SELECT PIDF
//            if (gamepad1.dpad_right && !lastRight) {
//                selected = (selected + 1) % 4;
//            }
//            lastRight = gamepad1.dpad_right;
//
//            if (gamepad1.dpad_left && !lastLeft) {
//                selected = (selected + 3) % 4;
//            }
//            lastLeft = gamepad1.dpad_left;
//
//            // CYCLE MULTIPLIER
//            if (gamepad1.right_bumper && !lastR1) {
//                multiplierIndex =
//                        (multiplierIndex + 1) % multipliers.length;
//            }
//            lastR1 = gamepad1.right_bumper;
//
//            double step = multipliers[multiplierIndex];
//
//            // INCREASE
//            if (gamepad1.dpad_up && !lastUp) {
//                changePIDF(step);
//            }
//            lastUp = gamepad1.dpad_up;
//
//            // DECREASE
//            if (gamepad1.dpad_down && !lastDown) {
//                changePIDF(-step);
//            }
//            lastDown = gamepad1.dpad_down;
//
//            // APPLY PIDF
//            if (gamepad1.cross && !lastCross) {
//                applyPIDF();
//            }
//            lastCross = gamepad1.cross;
//
//            // START / STOP
//            if (gamepad1.triangle && !lastTriangle) {
//                running = !running;
//            }
//            lastTriangle = gamepad1.triangle;
//
//            // RESET PIDF
//            if (gamepad1.square && !lastSquare) {
//
//                p = 1.1959;
//                i = 0.1196;
//                d = 0.0;
//                f = 11.9588;
//
//                applyPIDF();
//            }
//            lastSquare = gamepad1.square;
//
//            // EMERGENCY STOP
//            if (gamepad1.circle && !lastCircle) {
//                running = false;
//            }
//            lastCircle = gamepad1.circle;
//
//            // FLYWHEEL CONTROL
//            if (running) {
//                shooter.setVelocity(targetVelocity);
//            } else {
//                shooter.setVelocity(0);
//            }
//
//            double actualVelocity = shooter.getVelocity();
//
//            // TELEMETRY
//            telemetry.addLine("===== PIDF TUNER =====");
//
//            telemetry.addData("Selected",
//                    selected == 0 ? "P" :
//                            selected == 1 ? "I" :
//                                    selected == 2 ? "D" : "F");
//
//            telemetry.addLine("----------------------");
//
//            telemetry.addData("P", "%.4f", p);
//            telemetry.addData("I", "%.4f", i);
//            telemetry.addData("D", "%.4f", d);
//            telemetry.addData("F", "%.4f", f);
//
//            telemetry.addLine("----------------------");
//
//            telemetry.addData("Multiplier",
//                    "%.3f", step);
//
//            telemetry.addData("Target Velocity",
//                    "%.0f", targetVelocity);
//
//            telemetry.addData("Actual Velocity",
//                    "%.0f", actualVelocity);
//
//            telemetry.addData("Velocity Error",
//                    "%.0f", targetVelocity - actualVelocity);
//
//            telemetry.addData("Status",
//                    running ? "RUNNING" : "STOPPED");
//
//            telemetry.addData("PIDF",
//                    "Press X to apply");
//
//            telemetry.update();
//        }
//
//        shooter.setVelocity(0);
//    }
//
//    // CHANGE SELECTED PIDF VALUE
//    private void changePIDF(double amount) {
//
//        switch (selected) {
//
//            case 0:
//                p = Math.max(0, p + amount);
//                break;
//
//            case 1:
//                i = Math.max(0, i + amount);
//                break;
//
//            case 2:
//                d = Math.max(0, d + amount);
//                break;
//
//            case 3:
//                f = Math.max(0, f + amount);
//                break;
//        }
//    }
//
//    // APPLY PIDF TO MOTOR
//    private void applyPIDF() {
//
//        shooter.setPIDFCoefficients(
//                DcMotor.RunMode.RUN_USING_ENCODER,
//                new PIDFCoefficients(p, i, d, f)
//        );
//    }
//}
