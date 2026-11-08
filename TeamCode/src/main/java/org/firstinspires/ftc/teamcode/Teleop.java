
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {

    // Drive motors
    private DcMotor br;
    private DcMotor fr;
    private DcMotor fl;
    private DcMotor bl;

    // Mechanism motors
    private DcMotorEx transfur;
    private DcMotorEx shooter;
    private DcMotorEx intake;

    private IMU imu;

    // Flywheel settings
    private double flyspeed = 0;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void runOpMode() {

        // Hardware mapping
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");

        transfur = hardwareMap.get(DcMotorEx.class, "transfur");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        imu = hardwareMap.get(IMU.class, "imu");

        // Drive motor PIDF settings
        ((DcMotorEx) br).setVelocityPIDFCoefficients(
                1.0708, 0.1071, 0, 10.7087);

        ((DcMotorEx) fr).setVelocityPIDFCoefficients(
                1.0435, 0.1044, 0, 10.4254);

        ((DcMotorEx) fl).setVelocityPIDFCoefficients(
                1.785, 0.1078, 0, 10.70882);

        ((DcMotorEx) bl).setVelocityPIDFCoefficients(
                0.9999, 0.0999, 0, 9.999);

        // Motor directions
        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);

        // Encoder modes
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        transfur.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brake behavior
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transfur.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU initialization
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {

                // Field-centric driving
                // Hold left bumper for robot-relative driving
                drive(
                        -gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x,
                        !gamepad1.left_bumper
                );

                controls();
                shooter();
                intake();
                transfur();
                autoaim();
                telemetryUpdate();
            }
        }

        // Stop mechanism motors
        transfur.setVelocity(0);
        shooter.setVelocity(0);
        intake.setVelocity(0);
    }

    // Transfer motor
    private void transfur() {

        if (gamepad1.right_bumper) {
            transfur.setVelocity(2600);
        } else {
            transfur.setVelocity(0);
        }
    }

    // Shooter
    private void shooter() {
        shooter.setVelocity(flyspeed);
    }

    // Intake
    private void intake() {

        // TODO: Add intake voltage protection
    }

    // Auto aim
    private void autoaim() {

        // TODO: Add auto aim
    }

    // Driver controls
    private void controls() {

        // Reset IMU yaw
        if (gamepad1.ps) {
            imu.resetYaw();
        }

        // Increase flywheel speed once per press
        if (gamepad1.dpad_up && !lastDpadUp) {
            flyspeed += 100;
        }

        // Decrease flywheel speed once per press
        if (gamepad1.dpad_down && !lastDpadDown) {
            flyspeed -= 100;
        }

        // Limit flywheel speed
        flyspeed = Math.max(0, Math.min(2600, flyspeed));

        // Remember button states
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
    }

    // Telemetry
    private void telemetryUpdate() {
        telemetry.addLine("=== DRIVE VELOCITIES ===");

        telemetry.addData("FR Velocity", ((DcMotorEx) fr).getVelocity());
        telemetry.addData("FL Velocity", ((DcMotorEx) fl).getVelocity());
        telemetry.addData("BR Velocity", ((DcMotorEx) br).getVelocity());
        telemetry.addData("BL Velocity", ((DcMotorEx) bl).getVelocity());

        telemetry.addLine("=== MECHANISM VELOCITIES ===");

        telemetry.addData("Transfer Velocity", transfur.getVelocity());
        telemetry.addData("Shooter Target", flyspeed);
        telemetry.addData("Shooter Actual", shooter.getVelocity());
        telemetry.addData("Intake Velocity", intake.getVelocity());

        telemetry.addLine("=== DRIVER CONTROLS ===");

        telemetry.addLine("PS: Reset Yaw");
        telemetry.addLine("Left Bumper: Robot Relative");
        telemetry.addLine("Right Bumper: Transfer");
        telemetry.addLine("Dpad Up: Increase Flywheel");
        telemetry.addLine("Dpad Down: Decrease Flywheel");

        telemetry.update();
    }

    // Field-centric mecanum drive
    private void drive(float forward, float right, float rotate,
                       boolean fieldRelative) {

        // Field-relative conversion
        if (fieldRelative) {

            double theta = Math.atan2(forward, right) / Math.PI * 180;

            double r = Math.sqrt(forward * forward + right * right);

            theta = AngleUnit.DEGREES.normalize(
                    theta - imu.getRobotYawPitchRollAngles()
                            .getYaw(AngleUnit.DEGREES)
            );

            forward = (float) (
                    r * Math.sin(theta / 180 * Math.PI)
            );

            right = (float) (
                    r * Math.cos(theta / 180 * Math.PI)
            );
        }

        // Mecanum calculations
        float frontLeftPower = forward + right + rotate;
        float frontRightPower = forward - right - rotate;
        float backRightPower = forward + right - rotate;
        float backLeftPower = forward - right + rotate;

        // Normalize motor powers
        double maxPower = 1;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        double maxSpeed = 1.0;

        fl.setPower(maxSpeed * (frontLeftPower / maxPower));
        fr.setPower(maxSpeed * (frontRightPower / maxPower));
        bl.setPower(maxSpeed * (backLeftPower / maxPower));
        br.setPower(maxSpeed * (backRightPower / maxPower));
    }
}
