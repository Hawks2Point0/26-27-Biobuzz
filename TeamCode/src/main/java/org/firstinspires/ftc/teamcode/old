package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "teleop)", group = "Robot")
public class Teleop extends LinearOpMode {

  private DcMotor shooter;
  private DcMotor br;
  private DcMotor fr;
  private DcMotor fl;
  private DcMotor bl;
  private IMU imu;
  private DcMotor intake;
  private DcMotor transfur;

  int flyspeed;

  /**
   * This OpMode illustrates how to program your robot to drive field relative. This means
   * that the robot drives the direction you push the joystick regardless of the current orientation of the robot.
   *
   * This OpMode assumes that you have four mecanum wheels each on its own motor named:
   *  front_left_motor, front_right_motor, back_left_motor, back_right_motor
   *
   * and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
   */
  @Override
  public void runOpMode() {
    shooter = hardwareMap.get(DcMotor.class, "shooter");
    br = hardwareMap.get(DcMotor.class, "br");
    fr = hardwareMap.get(DcMotor.class, "fr");
    fl = hardwareMap.get(DcMotor.class, "fl");
    bl = hardwareMap.get(DcMotor.class, "bl");
    imu = hardwareMap.get(IMU.class, "imu");
    intake = hardwareMap.get(DcMotor.class, "intake");
    transfur = hardwareMap.get(DcMotor.class, "transfur");

    // Put initialization blocks here.
    ((DcMotorEx) shooter).setVelocityPIDFCoefficients(1.2412, 0.1214, 0, 12.359);
    ((DcMotorEx) br).setVelocityPIDFCoefficients(1.0708, 0.1071, 0, 10.7087);
    ((DcMotorEx) fr).setVelocityPIDFCoefficients(1.0435, 0.1044, 0, 10.4254);
    ((DcMotorEx) fl).setVelocityPIDFCoefficients(1.785, 0.1078, 0, 10.70882);
    ((DcMotorEx) bl).setVelocityPIDFCoefficients(0.9999, 0.0999, 0, 9.999);
    bl.setDirection(DcMotor.Direction.REVERSE);
    fl.setDirection(DcMotor.Direction.REVERSE);
    // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder wires, you should remove these
    fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    // Create a RevHubOrientationOnRobot object for use with an IMU in a REV Robotics Control
    // Hub or Expansion Hub, specifying the hub's orientation on the robot via the direction
    // that the REV Robotics logo is facing and the direction that the USB ports are facing.
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
    flyspeed = 1550;
    intake.setDirection(DcMotor.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        intake2();
        shooter2();
        drivetrain();
        telemerty();
      }
    }
  }

  /**
   * Describe this function...
   */
  private void intake2() {
    ((DcMotorEx) intake).setVelocity(2000);
  }

  /**
   * Describe this function...
   */
  private void drivetrain() {
    if (gamepad1.a) {
      imu.resetYaw();
    }
    drive_relative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
  }

  /**
   * Describe this function...
   */
  private void shooter2() {
    if (gamepad1.right_bumper) {
      ((DcMotorEx) shooter).setVelocity(flyspeed);
      if (flyspeed - 200 <= ((DcMotorEx) shooter).getVelocity()) {
        ((DcMotorEx) transfur).setVelocity(2600);
      } else {
        ((DcMotorEx) transfur).setVelocity(0);
      }
    } else {
      ((DcMotorEx) shooter).setVelocity(0);
      ((DcMotorEx) transfur).setVelocity(0);
    }
  }

  /**
   * Describe this function...
   */
  private void telemerty() {
    telemetry.addLine("Press A to reset Yaw");
    telemetry.addLine("Hold left bumper to drive in robot relative");
    telemetry.addData("FR", ((DcMotorEx) fr).getVelocity());
    telemetry.addData("FL", ((DcMotorEx) fl).getVelocity());
    telemetry.addData("BR", ((DcMotorEx) br).getVelocity());
    telemetry.addData("BL", ((DcMotorEx) bl).getVelocity());
    telemetry.addData("shooter diff", flyspeed - ((DcMotorEx) shooter).getVelocity());
    telemetry.update();
  }

  /**
   * This takes field relative, converts to field relative and drives the robot
   */
  private void drive_relative(float forward_relative, float right_relative, float rotate_relative) {
    double theta;
    double r;

    theta = Math.atan2(forward_relative, right_relative) / Math.PI * 180;
    r = Math.sqrt(forward_relative * forward_relative + right_relative * right_relative);
    theta = AngleUnit.DEGREES.normalize(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    drive(r * Math.sin(theta / 180 * Math.PI), r * Math.cos(theta / 180 * Math.PI), rotate_relative);
  }

  /**
   * Thanks to FTC16072 for sharing this code!
   */
  private void drive(double forward, double right, float rotate) {
    double maxPower;
    double frontLeftPower;
    double frontRightPower;
    double backRightPower;
    double backLeftPower;
    int maxSpeed;

    maxPower = 1;
    frontLeftPower = forward + right + rotate;
    frontRightPower = forward - (right + rotate);
    backRightPower = forward + (right - rotate);
    backLeftPower = forward - (right - rotate);
    // This is needed to make sure we don't pass > 1.0 to any wheel.
    // It allows us to keep all of the motors in proportion to what they should
    // be and not get clipped
    maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
    maxPower = Math.max(maxPower, Math.abs(frontRightPower));
    maxPower = Math.max(maxPower, Math.abs(backLeftPower));
    maxPower = Math.max(maxPower, Math.abs(backRightPower));
    // Change maxSpeed to be less than 1 for outreaches.  Do NOT change to be greater than 1
    maxSpeed = 1;
    fl.setPower(maxSpeed * (frontLeftPower / maxPower));
    fr.setPower(maxSpeed * (frontRightPower / maxPower));
    bl.setPower(maxSpeed * (backLeftPower / maxPower));
    br.setPower(maxSpeed * (backRightPower / maxPower));
  }
}
