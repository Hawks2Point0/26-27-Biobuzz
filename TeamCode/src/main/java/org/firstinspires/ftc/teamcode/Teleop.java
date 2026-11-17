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

@TeleOp(name = "Teleop", group = "Robot")
public class Teleop extends LinearOpMode {

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
    private static final double SPEED_STEP = 50;

    private boolean autoFlywheel = false;

    // Hybrid bang-bang shooter acceleration.
    // Full power is used while far below target, then velocity control
    // takes over near the target to reduce overshoot.
    private boolean shooterBoosting = false;
    private static final double SHOOTER_BOOST_POWER = 1.0;
    private static final double SHOOTER_BOOST_START_BELOW = 150;
    private static final double SHOOTER_BOOST_STOP_BELOW = 50;

    private boolean previousUp = false;
    private boolean previousDown = false;
    private boolean previousY = false;
    private boolean previousA = false;

    // =========================
    // LIMELIGHT SETTINGS
    // =========================

    private double limelightDistance = -1;
    private boolean limelightHasTarget = false;

    // These values must match your robot and the target you are detecting.
    private static final double CAMERA_HEIGHT_IN = 13.002;
    private static final double TARGET_HEIGHT_IN = 59.0;
    private static final double CAMERA_ANGLE_DEGREES = 45.0;

    private static final double AUTO_NEAR_DISTANCE_IN = 24.0;
    private static final double AUTO_FAR_DISTANCE_IN = 120.0;

    // Both are kept at your supplied values.
    // Change these to calibrated values if you want speed to vary by distance.
    private static final double AUTO_NEAR_SPEED = 1560;
    private static final double AUTO_FAR_SPEED = 1550;

    // =========================
    // INTAKE PROTECTION
    // =========================

    private boolean my_4_balls = false;

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
        limelight.pipelineSwitch(0);
        limelight.start();

        // =========================
        // DRIVETRAIN PIDF
        // =========================

        br.setVelocityPIDFCoefficients(
                1.0708, 0.1071, 0, 10.7087);

        fr.setVelocityPIDFCoefficients(
                1.0435, 0.1044, 0, 10.4254);

        fl.setVelocityPIDFCoefficients(
                1.785, 0.1078, 0, 10.70882);

        bl.setVelocityPIDFCoefficients(
                0.9999, 0.0999, 0, 9.999);

        shooter.setVelocityPIDFCoefficients(
                1.1959, 0.1196, 0, 11.9588);

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
                DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT);

        shooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT);

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

        if (autoFlywheel) {

            // Automatically choose flywheel speed from Limelight distance.
            if (limelightHasTarget && limelightDistance > 0) {
                flyspeed = calculateAutoFlywheelSpeed(limelightDistance);
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

        // Speed limits
        flyspeed = Math.max(
                MIN_FLYSPEED,
                Math.min(MAX_FLYSPEED, flyspeed)
        );

        // Read the measured flywheel speed.
        double actualShooterVelocity = shooter.getVelocity();

        // Hybrid bang-bang acceleration:
        // Apply full power while well below target, then hand control back
        // to the motor velocity controller near the target.
        if (!shooterBoosting
                && actualShooterVelocity < flyspeed - SHOOTER_BOOST_START_BELOW) {
            shooterBoosting = true;
        } else if (shooterBoosting
                && actualShooterVelocity >= flyspeed - SHOOTER_BOOST_STOP_BELOW) {
            shooterBoosting = false;
        }

        if (shooterBoosting) {
            shooter.setPower(SHOOTER_BOOST_POWER);
        } else {
            shooter.setVelocity(flyspeed);
        }

        // Require the shooter to be 100 velocity units ABOVE the target
        // before allowing the transfer to feed.
        boolean shooterReady = actualShooterVelocity - 100 >= flyspeed;

        // Transfer only while the shooter is at or above its target velocity.
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
    // DRIVETRAIN
    // =========================

    private void drivetrain() {

        if (gamepad1.ps) {
            imu.resetYaw();
        }

        if (gamepad1.options) {

            // Robot-relative driving
            drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

        } else {

            // Field-relative driving
            drive_relative(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
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
                autoFlywheel ? "AUTO (LIMELIGHT)" : "MANUAL"
        );

        telemetry.addData("Target Speed", flyspeed);

        telemetry.addData(
                "Actual Speed",
                shooter.getVelocity()
        );

        telemetry.addData(
                "Shooter Control",
                shooterBoosting ? "BANG-BANG BOOST" : "VELOCITY HOLD"
        );

        telemetry.addData(
                "Shooter Ready",
                shooter.getVelocity() - 100 >= flyspeed ? "YES" : "WAITING"
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
        telemetry.addData(
                "Limelight Status",
                limelightHasTarget ? "Target detected" : "No valid target"
        );

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
