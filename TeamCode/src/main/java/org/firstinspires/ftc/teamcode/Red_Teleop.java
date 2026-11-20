
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Red Teleop", group = "Robot")
public class Red_Teleop extends LinearOpMode {

    // =========================
    // HARDWARE
    // =========================

    private DcMotorEx br;
    private DcMotorEx fr;
    private DcMotorEx fl;
    private DcMotorEx bl;

    private DcMotorEx intake;
    private DcMotorEx transfur;
    private DcMotorEx shooter;

    private Limelight3A limelight;
    private IMU imu;

    // =========================
    // FLYWHEEL SETTINGS
    // =========================

    private double flyspeed = 1550;

    private static final double MIN_FLYSPEED = 0;
    private static final double MAX_FLYSPEED = 2600;
    private static final double SPEED_STEP = 100;

    private boolean autoFlywheel = false;

    private boolean shooterBoosting = false;

    private static final double SHOOTER_BOOST_POWER = 1.0;
    private static final double SHOOTER_BOOST_START_BELOW = 75;
    private static final double SHOOTER_BOOST_STOP_BELOW = 45;

    private boolean previousUp = false;
    private boolean previousDown = false;
    private boolean previousY = false;
    private boolean previousA = false;

    // =========================
    // LIMELIGHT SETTINGS
    // =========================

    private double limelightDistance = -1;
    private boolean limelightHasTarget = false;
    private double limelightTx = 0;

    private static final double CAMERA_HEIGHT_IN = 13.002;
    private static final double TARGET_HEIGHT_IN = 59.0;
    private static final double CAMERA_ANGLE_DEGREES = 45.0;

    private static final double AUTO_NEAR_DISTANCE_IN = 24.0;
    private static final double AUTO_FAR_DISTANCE_IN = 120.0;

    private static final double AUTO_NEAR_SPEED = 1560;
    private static final double AUTO_FAR_SPEED = 1550;

    // =========================
    // AUTO AIM SETTINGS
    // =========================

    private boolean IsAutoAming = false;

    private double autoAimPower = 0;

    private double flywheel_tps = 1550;
    private double hood_angle = 0;

    // =========================
    // INTAKE PROTECTION
    // =========================

    private boolean my_4_balls = false;

    // =========================
    // RUN OP MODE
    // =========================

    @Override
    public void runOpMode() {

        // =========================
        // HARDWARE MAPPING
        // =========================

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        br = hardwareMap.get(DcMotorEx.class, "br");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        bl = hardwareMap.get(DcMotorEx.class, "bl");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfur = hardwareMap.get(DcMotorEx.class, "transfur");

        imu = hardwareMap.get(IMU.class, "imu");

        // =========================
        // LIMELIGHT SETUP
        // =========================

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(1);
        limelight.start();

        // =========================
        // DRIVETRAIN PIDF
        // =========================

        br.setVelocityPIDFCoefficients(
                1.0708, 0.1071, 0, 10.7087
        );

        fr.setVelocityPIDFCoefficients(
                1.0435, 0.1044, 0, 10.4254
        );

        fl.setVelocityPIDFCoefficients(
                1.785, 0.1078, 0, 10.70882
        );

        bl.setVelocityPIDFCoefficients(
                0.9999, 0.0999, 0, 9.999
        );

        shooter.setVelocityPIDFCoefficients(
                1.1959, 0.1196, 0, 11.9588
        );

        // =========================
        // MOTOR DIRECTIONS
        // =========================

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.REVERSE);

        // =========================
        // MOTOR MODES
        // =========================

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfur.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // =========================
        // ZERO POWER BEHAVIOR
        // =========================

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transfur.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        shooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        // =========================
        // INTAKE CURRENT LIMIT
        // =========================

        intake.setCurrentAlert(4, CurrentUnit.AMPS);

        // =========================
        // IMU INITIALIZATION
        // =========================

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        telemetry.addLine("Robot Initialized");
        telemetry.addLine("Limelight starting...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            limelight.stop();
            return;
        }

        imu.resetYaw();

        // =========================
        // MAIN LOOP
        // =========================

        try {

            while (opModeIsActive()) {

                updateLimelightDistance();

                auto_aim();

                intake2();
                shooter2();
                drivetrain();

                telemetryUpdate();
            }

        } finally {

            // Stop all motors

            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);

            shooter.setVelocity(0);
            intake.setVelocity(0);
            transfur.setVelocity(0);

            limelight.stop();
        }
    }

    // =========================
    // INTAKE
    // =========================

    private void intake2() {

        if (gamepad1.circle) {
            my_4_balls = false;
        }

        // Reverse intake

        if (gamepad1.share) {
            intake.setVelocity(-2450);
            return;
        }

        // Shooting resets intake current-limit lockout

        if (gamepad1.right_bumper) {
            my_4_balls = false;

        } else if (intake.isOverCurrent()) {
            my_4_balls = true;
        }

        // Stop intake at current limit

        if (my_4_balls) {
            intake.setVelocity(0);

        } else {
            intake.setVelocity(2400);
        }
    }

    // =========================
    // SHOOTER AND TRANSFER
    // =========================

    private void shooter2() {

        // Toggle Auto / Manual with Triangle

        boolean a = gamepad1.triangle;

        if (a && !previousA) {
            autoFlywheel = !autoFlywheel;
        }

        previousA = a;

        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean y = gamepad1.square;

        // =========================
        // FLYWHEEL SPEED CONTROL
        // =========================

        if (IsAutoAming) {

            // Auto aim sets flyspeed using Limelight distance.
            // Do not overwrite it with manual controls.

        } else if (autoFlywheel) {

            // Existing Limelight distance-based speed mode

            if (limelightHasTarget && limelightDistance > 0) {

                flyspeed = calculateAutoFlywheelSpeed(
                        limelightDistance
                );
            }

        } else {

            // Manual speed control

            if (up && !previousUp) {
                flyspeed += SPEED_STEP;
            }

            if (down && !previousDown) {
                flyspeed -= SPEED_STEP;
            }

            if (y && !previousY) {
                flyspeed = 1550;
            }
        }

        previousUp = up;
        previousDown = down;
        previousY = y;

        // =========================
        // SPEED LIMITS
        // =========================

        flyspeed = Math.max(
                MIN_FLYSPEED,
                Math.min(MAX_FLYSPEED, flyspeed)
        );

        // =========================
        // SHOOTER CONTROL
        // =========================

        double actualShooterVelocity = shooter.getVelocity();

        if (!shooterBoosting
                && actualShooterVelocity
                < flyspeed - SHOOTER_BOOST_START_BELOW) {

            shooterBoosting = true;

        } else if (shooterBoosting
                && actualShooterVelocity
                >= flyspeed - SHOOTER_BOOST_STOP_BELOW) {

            shooterBoosting = false;
        }

        if (shooterBoosting) {

            shooter.setPower(SHOOTER_BOOST_POWER);

        } else {

            shooter.setVelocity(flyspeed);
        }

        // =========================
        // SHOOTER READY CHECK
        // =========================

        boolean shooterReady =
                actualShooterVelocity - 100 >= flyspeed;

        // =========================
        // TRANSFER
        // =========================

        if (gamepad1.right_bumper && shooterReady) {

            transfur.setVelocity(2600);

        } else {

            transfur.setVelocity(0);
        }
    }

    // =========================
    // LIMELIGHT AUTO SPEED
    // =========================

    private double calculateAutoFlywheelSpeed(double distance) {

        double limitedDistance = Math.max(
                AUTO_NEAR_DISTANCE_IN,
                Math.min(AUTO_FAR_DISTANCE_IN, distance)
        );

        double distanceRange =
                AUTO_FAR_DISTANCE_IN - AUTO_NEAR_DISTANCE_IN;

        if (distanceRange <= 0) {
            return AUTO_NEAR_SPEED;
        }

        double ratio =
                (limitedDistance - AUTO_NEAR_DISTANCE_IN)
                        / distanceRange;

        return AUTO_NEAR_SPEED
                + ratio * (AUTO_FAR_SPEED - AUTO_NEAR_SPEED);
    }

    // =========================
    // LIMELIGHT DISTANCE TRACKING
    // =========================

    private void updateLimelightDistance() {

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            limelightTx = result.getTx();

            double ty = result.getTy();

            double angleDegrees =
                    CAMERA_ANGLE_DEGREES + ty;

            if (angleDegrees > 0.1 && angleDegrees < 89.9) {

                double angleRadians =
                        Math.toRadians(angleDegrees);

                double distance =
                        (TARGET_HEIGHT_IN - CAMERA_HEIGHT_IN)
                                / Math.tan(angleRadians);

                if (distance > 0 && Double.isFinite(distance)) {

                    limelightDistance = distance;
                    limelightHasTarget = true;
                    return;
                }
            }
        }

        limelightHasTarget = false;
        limelightDistance = -1;
    }

    // =========================
    // LIMELIGHT AUTO AIM
    // =========================

    private void auto_aim() {

        if (gamepad1.left_bumper) {

            IsAutoAming = true;

            // Limelight was already started during initialization.
            // Do not restart it every loop.

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                double tx = result.getTx();

                // =========================
                // AUTO AIM POWER
                // =========================

                double AutoPower = Math.abs(tx) / 65.0;

                telemetry.addData("Auto Power", AutoPower);
                telemetry.addData("TX", tx);
                telemetry.addData("TY", result.getTy());
                telemetry.addData("TA", result.getTa());

                telemetry.addLine(" ");

                if (AutoPower < 0.175) {
                    AutoPower = 0.15;
                }

                // =========================
                // HORIZONTAL AUTO AIM
                // =========================

                if (tx > 1.5) {

                    autoAimPower = AutoPower;

                } else if (tx < -1.5) {

                    autoAimPower = -AutoPower;

                } else {

                    autoAimPower = 0;
                }

                // =========================
                // DISTANCE CALCULATION
                // =========================

                double Percent = result.getTa();

                double Distance =
                        160.0 * Math.pow(Percent + 0.1, -0.7) - 2.0;

                telemetry.addData("Distance", Distance);

                // =========================
                // DISTANCE-BASED FLYWHEEL
                // =========================

                if (Double.isFinite(Distance) && Distance > 0) {

                    if (Distance < 80) {

                        flywheel_tps = 2.0 * Distance + 950;
                        hood_angle = 0.0;

                    } else if (Distance <= 180) {

                        flywheel_tps = 2.0 * Distance + 980;
                        hood_angle = 0.0;

                    } else {

                        flywheel_tps =
                                1150.0 + 125.0 * Math.log(Distance);

                        hood_angle = 0.382;
                    }

                    // Use the existing shooter motor

                    flyspeed = flywheel_tps;
                }

                telemetry.addData(
                        "TPS From Distance",
                        flywheel_tps
                );

                telemetry.addData(
                        "Hood Angle",
                        hood_angle
                );

            } else {

                // No valid target: stop auto turning

                autoAimPower = 0;

                telemetry.addLine("No valid Limelight target");
            }

        } else {

            IsAutoAming = false;
            autoAimPower = 0;
        }
    }

    // =========================
    // DRIVETRAIN
    // =========================

    private void drivetrain() {

        if (gamepad1.ps) {
            imu.resetYaw();
        }

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;

        double rotate;

        // L1 held: Limelight controls turning.
        // L1 released: right stick controls turning.

        if (IsAutoAming) {

            rotate = autoAimPower;

        } else {

            rotate = gamepad1.right_stick_x;
        }

        if (gamepad1.options) {

            // Robot-relative driving

            drive(
                    forward,
                    right,
                    rotate
            );

        } else {

            // Field-relative driving

            drive_relative(
                    forward,
                    right,
                    rotate
            );
        }
    }

    // =========================
    // TELEMETRY
    // =========================

    private void telemetryUpdate() {

        telemetry.addLine("=== FLYWHEEL ===");

        telemetry.addData(
                "Flywheel Mode",
                IsAutoAming ? "LIMELIGHT AUTO AIM"
                        : autoFlywheel ? "AUTO" : "MANUAL"
        );

        telemetry.addData("Target Speed", flyspeed);
        telemetry.addData("Actual Speed", shooter.getVelocity());

        telemetry.addData(
                "Shooter Control",
                shooterBoosting ? "BANG-BANG BOOST" : "VELOCITY HOLD"
        );

        telemetry.addData(
                "Shooter Ready",
                shooter.getVelocity() - 100 >= flyspeed
                        ? "YES" : "WAITING"
        );

        telemetry.addData(
                "Speed Difference",
                flyspeed - shooter.getVelocity()
        );

        telemetry.addData(
                "Shooter Current",
                shooter.getCurrent(CurrentUnit.AMPS)
        );

        telemetry.addLine("================");

        telemetry.addLine("=== LIMELIGHT ===");

        telemetry.addData("Limelight Target", limelightHasTarget);
        telemetry.addData("Distance (in)", limelightDistance);
        telemetry.addData("TX", limelightTx);

        telemetry.addData("Auto Aim", IsAutoAming);
        telemetry.addData("Auto Aim Power", autoAimPower);

        telemetry.addData("Auto Flywheel TPS", flywheel_tps);
        telemetry.addData("Hood Angle", hood_angle);

        telemetry.addLine("================");

        telemetry.addData("FR Velocity", fr.getVelocity());
        telemetry.addData("FL Velocity", fl.getVelocity());
        telemetry.addData("BR Velocity", br.getVelocity());
        telemetry.addData("BL Velocity", bl.getVelocity());

        telemetry.addData(
                "Intake Velocity",
                intake.getVelocity()
        );

        telemetry.addData(
                "Intake Amps",
                intake.getCurrent(CurrentUnit.AMPS)
        );

        telemetry.addData(
                "Intake Current Lockout",
                my_4_balls
        );

        telemetry.addData(
                "Transfer Velocity",
                transfur.getVelocity()
        );

        telemetry.addData(
                "Yaw",
                imu.getRobotYawPitchRollAngles()
                        .getYaw(AngleUnit.DEGREES)
        );

        telemetry.update();
    }

    // =========================
    // FIELD CENTRIC
    // =========================

    private void drive_relative(
            double forward_relative,
            double right_relative,
            double rotate_relative) {

        double theta = Math.atan2(
                forward_relative,
                right_relative
        ) / Math.PI * 180;

        double r = Math.sqrt(
                forward_relative * forward_relative
                        + right_relative * right_relative
        );

        theta = AngleUnit.DEGREES.normalize(
                theta - imu.getRobotYawPitchRollAngles()
                        .getYaw(AngleUnit.DEGREES)
        );

        drive(
                r * Math.sin(theta / 180 * Math.PI),
                r * Math.cos(theta / 180 * Math.PI),
                rotate_relative
        );
    }

    // =========================
    // MECANUM DRIVE
    // =========================

    private void drive(
            double forward,
            double right,
            double rotate) {

        double frontLeftPower;
        double frontRightPower;
        double backRightPower;
        double backLeftPower;

        frontLeftPower = forward + right + rotate;
        frontRightPower = forward - right - rotate;
        backRightPower = forward + right - rotate;
        backLeftPower = forward - right + rotate;

        double maxPower = Math.max(
                1.0,
                Math.max(
                        Math.max(
                                Math.abs(frontLeftPower),
                                Math.abs(frontRightPower)
                        ),
                        Math.max(
                                Math.abs(backLeftPower),
                                Math.abs(backRightPower)
                        )
                )
        );

        fl.setPower(frontLeftPower / maxPower);
        fr.setPower(frontRightPower / maxPower);
        bl.setPower(backLeftPower / maxPower);
        br.setPower(backRightPower / maxPower);
    }
}
