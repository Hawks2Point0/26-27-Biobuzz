
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Teleop", group = "Robot")
public class Teleop extends LinearOpMode {

    private DcMotorEx shooter;
    private DcMotorEx br;
    private DcMotorEx fr;
    private DcMotorEx fl;
    private DcMotorEx bl;
    private DcMotorEx intake;
    private DcMotorEx transfur;

    private IMU imu;

    private int flyspeed = 1550;

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
        // IMU INITIALIZATION
        // =========================

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        telemetry.addLine("Robot Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {

                // Shooter always runs
                shooter.setVelocity(flyspeed);

                // Run subsystems
                intake2();
                shooter2();
                drivetrain();
                telemetryUpdate();
            }
        }

        // =========================
        // STOP ALL MOTORS
        // =========================

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        shooter.setVelocity(0);
        intake.setVelocity(0);
        transfur.setVelocity(0);
    }

    // =========================
    // INTAKE
    // =========================

    private void intake2() {

        intake.setVelocity(2200);

    }

    // =========================
    // DRIVETRAIN
    // =========================

    private void drivetrain() {

        if (gamepad1.ps) {
            imu.resetYaw();
        }

        if (gamepad1.left_bumper) {

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
    // SHOOTER AND TRANSFER
    // =========================

    private void shooter2() {

        // Keep shooter running
        shooter.setVelocity(flyspeed);

        // Transfer activates immediately when shooter is ready
        if (gamepad1.right_bumper) {

            transfur.setVelocity(2600);

        } else {

            transfur.setVelocity(0);
        }
    }

    // =========================
    // TELEMETRY
    // =========================

    private void telemetryUpdate() {

        telemetry.addLine("Press PS to reset Yaw");
        telemetry.addLine("Hold LEFT BUMPER for robot-relative driving");
        telemetry.addLine("Shooter always running");
        telemetry.addLine("Hold RIGHT BUMPER to shoot");

        telemetry.addData("FR Velocity", fr.getVelocity());
        telemetry.addData("FL Velocity", fl.getVelocity());
        telemetry.addData("BR Velocity", br.getVelocity());
        telemetry.addData("BL Velocity", bl.getVelocity());

        telemetry.addData(
                "Shooter Velocity",
                shooter.getVelocity()
        );

        telemetry.addData(
                "Shooter Difference",
                flyspeed - shooter.getVelocity()
        );

        telemetry.addData(
                "Intake Velocity",
                intake.getVelocity()
        );

        telemetry.addData(
                "Intake Amps",
                intake.getCurrent(CurrentUnit.AMPS)
        );

        telemetry.addData(
                "Transfer Velocity",
                transfur.getVelocity()
        );

        telemetry.update();
    }

    // =========================
    // FIELD RELATIVE DRIVE
    // =========================

    private void drive_relative(
            double forward_relative,
            double right_relative,
            double rotate_relative) {

        double theta;
        double r;

        theta = Math.atan2(
                forward_relative,
                right_relative
        ) / Math.PI * 180;

        r = Math.sqrt(
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
