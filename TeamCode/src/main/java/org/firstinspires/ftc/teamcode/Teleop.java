
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

    private boolean my_3_balls = false;
    private boolean previousShootButton = false;

    @Override
    public void runOpMode() {

        // Hardware mapping
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        br = hardwareMap.get(DcMotorEx.class, "br");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        bl = hardwareMap.get(DcMotorEx.class, "bl");

        imu = hardwareMap.get(IMU.class, "imu");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfur = hardwareMap.get(DcMotorEx.class, "transfur");

        // Shooter PIDF
        shooter.setVelocityPIDFCoefficients(
                1.2412, 0.1214, 0, 12.359
        );

        // Drivetrain PIDF
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

        // Motor directions
        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.REVERSE);

        // Drivetrain modes
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brake behavior
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Intake current alert
        intake.setCurrentAlert(5.25, CurrentUnit.AMPS);

        // IMU initialization
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );

        telemetry.addLine("Robot Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            drivetrain();

            intake2();

            shooter2();

            telemetryUpdate();
        }

        // Stop motors
        shooter.setVelocity(0);
        transfur.setVelocity(0);
        intake.setVelocity(0);

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    // INTAKE
    private void intake2() {

        boolean shooting = gamepad1.right_bumper;

        // Reset intake current lockout when shooting starts
        if (shooting && !previousShootButton) {
            my_3_balls = false;
        }

        previousShootButton = shooting;

        // Check current limit
        if (intake.isOverCurrent()) {
            my_3_balls = true;
        }

        // Stop intake when current limit is reached
        if (my_3_balls) {
            intake.setVelocity(0);
        } else {
            intake.setVelocity(2000);
        }
    }

    // SHOOTER AND TRANSFER
    private void shooter2() {

        if (gamepad1.right_bumper) {

            shooter.setVelocity(flyspeed);

            // Wait until shooter is close to target speed
            if (shooter.getVelocity() >= flyspeed - 200) {
                transfur.setVelocity(2600);
            } else {
                transfur.setVelocity(0);
            }

        } else {

            shooter.setVelocity(0);
            transfur.setVelocity(0);
        }
    }

    // DRIVETRAIN
    private void drivetrain() {

        if (gamepad1.a) {
            imu.resetYaw();
        }

        drive_relative(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );
    }

    // TELEMETRY
    private void telemetryUpdate() {

        telemetry.addLine("SHOOTER");

        telemetry.addData("Target Velocity", flyspeed);
        telemetry.addData("Actual Velocity", "%.1f", shooter.getVelocity());
        telemetry.addData(
                "Shooter Difference",
                "%.1f",
                flyspeed - shooter.getVelocity()
        );

        telemetry.addLine("INTAKE");

        telemetry.addData(
                "Intake Current",
                "%.2f A",
                intake.getCurrent(CurrentUnit.AMPS)
        );

        telemetry.addData("Intake Locked", my_3_balls);

        telemetry.addLine("TRANSFER");

        telemetry.addData("Transfer Velocity", transfur.getVelocity());

        telemetry.update();
    }

    // FIELD-CENTRIC DRIVE
    private void drive_relative(
            double forward_relative,
            double right_relative,
            double rotate_relative
    ) {

        double theta = Math.atan2(
                forward_relative,
                right_relative
        ) / Math.PI * 180;

        double r = Math.sqrt(
                forward_relative * forward_relative +
                        right_relative * right_relative
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

    // MECANUM DRIVE
    private void drive(
            double forward,
            double right,
            double rotate
    ) {

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
