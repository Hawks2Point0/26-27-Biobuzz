
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "red Teleop", group = "Robot")
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

    private Servo hood;

    private Limelight3A limelight;
    private IMU imu;

    // =========================
    // HOOD SERVO
    // =========================

    private double hoodPosition = 0.26;

    private static final double HOOD_STEP = 0.01;
    private static final double HOOD_MIN = 0.0;
    private static final double HOOD_MAX = 0.5;

    private boolean previousHoodUp = false;
    private boolean previousHoodDown = false;

    // =========================
    // FLYWHEEL SETTINGS
    // =========================

    private double flyspeed = 1650;

    private static final double MIN_FLYSPEED = 0;
    private static final double MAX_FLYSPEED = 2600;
    private static final double SPEED_STEP = 25;

    private boolean previousUp = false;
    private boolean previousDown = false;
    private boolean previousY = false;

    // =========================
    // SHOOTER PIDF
    // =========================

    private static final double SHOOTER_P = 61.8;
    private static final double SHOOTER_I = 0.1;
    private static final double SHOOTER_D = 0.1460;
    private static final double SHOOTER_F = 11.4608;

    // =========================
    // LIMELIGHT AUTO AIM
    // =========================

    private boolean IsAutoAming = false;
    private boolean limelightHasTarget = false;

    private double limelightTx = 0;
    private double AutoPower = 0;

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

        hood = hardwareMap.get(Servo.class, "hood");

        imu = hardwareMap.get(IMU.class, "imu");

        // =========================
        // HOOD INITIAL POSITION
        // =========================

        hood.setPosition(hoodPosition);

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
                1.0708, 0.1071, 0, 10.7087);

        fr.setVelocityPIDFCoefficients(
                1.0435, 0.1044, 0, 10.4254);

        fl.setVelocityPIDFCoefficients(
                1.785, 0.1078, 0, 10.70882);

        bl.setVelocityPIDFCoefficients(
                0.9999, 0.0999, 0, 9.999);

        // =========================
        // FLYWHEEL PIDF
        // =========================

        shooter.setVelocityPIDFCoefficients(
                SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);

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

        intake.setCurrentAlert(3.45, CurrentUnit.AMPS);

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
        telemetry.addLine("Hood Servo Ready");
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

                updateLimelight();

                hoodControl();

                intake2();

                shooter2();

                drivetrain();

                telemetryUpdate();
            }

        } finally {

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
    // HOOD CONTROL
    // =========================

    private void hoodControl() {

        boolean hoodUp = gamepad1.dpad_right;
        boolean hoodDown = gamepad1.dpad_left;

        // Raise hood
        if (hoodUp && !previousHoodUp) {
            hoodPosition += HOOD_STEP;
        }

        // Lower hood
        if (hoodDown && !previousHoodDown) {
            hoodPosition -= HOOD_STEP;
        }

        previousHoodUp = hoodUp;
        previousHoodDown = hoodDown;

        // Servo limits
        hoodPosition = Math.max(
                HOOD_MIN,
                Math.min(HOOD_MAX, hoodPosition)
        );

        hood.setPosition(hoodPosition);

        telemetry.addData("Hood Position", "%.2f", hoodPosition);
    }

    // =========================
    // LIMELIGHT UPDATE
    // =========================

    private void updateLimelight() {

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            limelightTx = result.getTx();
            limelightHasTarget = true;

        } else {

            limelightTx = 0;
            limelightHasTarget = false;
        }
    }

    // =========================
    // HORIZONTAL AUTO AIM
    // =========================

    private void auto_aim() {

        if (gamepad1.left_bumper) {

            IsAutoAming = true;

            if (limelightHasTarget) {

                double tx = limelightTx;

                AutoPower = Math.abs(tx) / 43.5;

                if (AutoPower < 0.175) {
                    AutoPower = 0.125;
                }

                telemetry.addData("Auto Aim", "Active");
                telemetry.addData("Auto Power", AutoPower);
                telemetry.addData("TX", tx);

            } else {

                AutoPower = 0;

                telemetry.addData("Auto Aim", "No Target");
            }

        } else {

            IsAutoAming = false;
            AutoPower = 0;
        }
    }

    // =========================
    // INTAKE
    // =========================

    private void intake2() {

        if (gamepad1.circle) {
            my_4_balls = false;
        }

        if (gamepad1.share) {
            intake.setVelocity(-2450);
            return;
        }

        if (gamepad1.right_bumper) {
            my_4_balls = false;

        } else if (intake.isOverCurrent()) {
            my_4_balls = true;
        }

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

        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean y = gamepad1.square;

        if (up && !previousUp) {
            flyspeed += SPEED_STEP;
        }

        if (down && !previousDown) {
            flyspeed -= SPEED_STEP;
        }

        if (y && !previousY) {
            flyspeed = 1550;
        }

        previousUp = up;
        previousDown = down;
        previousY = y;

        flyspeed = Math.max(
                MIN_FLYSPEED,
                Math.min(MAX_FLYSPEED, flyspeed)
        );

        shooter.setVelocity(flyspeed);

        double actualShooterVelocity = shooter.getVelocity();

        boolean shooterReady = actualShooterVelocity >= flyspeed - 100;

        if (gamepad1.right_bumper && shooterReady) {

            transfur.setVelocity(2600);

        } else {

            transfur.setVelocity(0);
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
        double rotate = gamepad1.right_stick_x;

        auto_aim();

        if (IsAutoAming && limelightHasTarget) {

            if (limelightTx > 1.25) {

                rotate = AutoPower;

            } else if (limelightTx < -1.25) {

                rotate = -AutoPower;

            } else {

                rotate = 0;
            }
        }

        if (gamepad1.options) {

            drive(
                    forward,
                    right,
                    rotate
            );

        } else {

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

        telemetry.addData("Target Speed", flyspeed);
        telemetry.addData("Actual Speed", shooter.getVelocity());

        telemetry.addData(
                "Shooter Control",
                "VELOCITY HOLD"
        );

        telemetry.addData("Shooter Power", shooter.getPower());

        telemetry.addData(
                "Shooter Ready",
                shooter.getVelocity() >= flyspeed - 100
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

        telemetry.addData("Auto Aiming", IsAutoAming);
        telemetry.addData("Limelight Target", limelightHasTarget);
        telemetry.addData("TX", limelightTx);
        telemetry.addData("Auto Power", AutoPower);

        telemetry.addLine("================");

        telemetry.addData("FR Velocity", fr.getVelocity());
        telemetry.addData("FL Velocity", fl.getVelocity());
        telemetry.addData("BR Velocity", br.getVelocity());
        telemetry.addData("BL Velocity", bl.getVelocity());

        telemetry.addData("Intake Velocity", intake.getVelocity());

        telemetry.addData(
                "Intake Amps",
                intake.getCurrent(CurrentUnit.AMPS)
        );

        telemetry.addData("Intake Current Lockout", my_4_balls);

        telemetry.addData(
                "Transfer Velocity",
                transfur.getVelocity()
        );

        telemetry.addData(
                "Hood Position",
                hoodPosition
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
