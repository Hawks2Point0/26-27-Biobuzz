package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunLEDStick;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "blue Teleop", group = "Robot")
public class Blue_Teleop extends LinearOpMode {

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

    private static final double HOOD_STEP = 0.025;
    private static final double HOOD_MIN = 0.0;
    private static final double HOOD_MAX = 0.5;

    // Hood presets (Cross = low, Triangle = high)
    private static final double HOOD_PRESET_LOW = 0.25;
    private static final double HOOD_PRESET_HIGH = 0.35;

    private boolean previousHoodUp = false;
    private boolean previousHoodDown = false;
    private boolean previousCross = false;
    private boolean previousTriangle = false;

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

    private static final double SHOOTER_P = 81.8;
    private static final double SHOOTER_I = 0.2;
    private static final double SHOOTER_D = 0;
    private static final double SHOOTER_F = 11.9588;

    // =========================
    // SPIN-UP TIMER
    // =========================
    // Times how long the flywheel takes to get "ready" (within 100 ticks/sec of target).
    // Starts whenever the shooter is NOT ready (start-up, speed change, dip after a shot)
    // and stops the moment it becomes ready.

    private boolean spinUpTiming = false;
    private long spinUpStartNs = 0;
    private double spinUpSeconds = 0;      // live while spinning up, final once ready
    private double lastSpinUpSeconds = 0;  // most recent completed spin-up

    // =========================
    // LIMELIGHT AUTO AIM
    // =========================

    private boolean IsAutoAming = false;
    private boolean limelightHasTarget = false;

    private double limelightTx = 0;
    private double AutoPower = 0;

    // Auto-aim PIDF tuning. Output is drivetrain turn power.
    private static final double AIM_P = 0.023;
    private static final double AIM_I = 0.00015;
    private static final double AIM_D = 0.001;
    private static final double AIM_F = 0.08;
    private static final double AIM_TOLERANCE_DEG = 1.25;
    private static final double AIM_MAX_POWER = 0.50;
    private static final double AIM_MIN_POWER = 0.10;
    private static final double AIM_INTEGRAL_LIMIT = 80.0;

    private double aimIntegral = 0;
    private double previousAimError = 0;
    private long previousAimTimeNs = 0;

    // =========================
    // LIMELIGHT DISTANCE
    // =========================
    // TUNE THESE THREE VALUES FOR YOUR ROBOT (all in inches / degrees):
    //  - CAMERA_HEIGHT_IN:       floor to the center of the Limelight lens
    //  - TARGET_HEIGHT_IN:       floor to the center of the AprilTag / target
    //  - CAMERA_MOUNT_ANGLE_DEG: how far the camera is tilted UP from horizontal
    private static final double CAMERA_HEIGHT_IN = 10.0;
    private static final double TARGET_HEIGHT_IN = 29.5;
    private static final double CAMERA_MOUNT_ANGLE_DEG = 20.0;

    // Distance from the camera to the shooter / robot center (forward = positive).
    // Set to 0 if you just want camera-to-target distance.
    private static final double CAMERA_FORWARD_OFFSET_IN = 0.0;

    // Smoothing: 1.0 = no smoothing, lower = smoother but slower to react
    private static final double DISTANCE_FILTER_ALPHA = 0.3;

    private double limelightTy = 0;
    private double distanceInches = 0;          // raw
    private double filteredDistanceInches = 0;  // smoothed
    private boolean hasDistance = false;

    // =========================
    // INTAKE PROTECTION
    // =========================

    private boolean my_4_balls = false;

    // Intake-full threshold is adjusted for battery voltage:
    //   threshold = base * (NOMINAL_VOLTAGE / battery) ^ VOLTAGE_COMP_GAIN
    //   gain  1.0 = threshold goes UP as the battery drops (current rises on a weak battery)
    //   gain  0.0 = no compensation (fixed threshold)
    //   gain -1.0 = threshold goes DOWN as the battery drops
    // Watch "Intake Amps" with a full load on a fresh and a worn battery, then tune.
    private static final double INTAKE_STOP_AMPS_AT_NOMINAL = 3.45;  // tune at ~12V battery
    private static final double VOLTAGE_COMP_GAIN = 1.0;
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double MIN_VOLTAGE_CLAMP = 9.0;
    private static final double MAX_VOLTAGE_CLAMP = 14.0;

    private VoltageSensor voltageSensor;
    private double batteryVoltage = NOMINAL_VOLTAGE;
    private double intakeStopAmps = INTAKE_STOP_AMPS_AT_NOMINAL;

    // =========================
    // LED STICK (SparkFun Qwiic)
    // =========================

    private SparkFunLEDStick led;

    private static final int LED_BRIGHTNESS = 10;  // 0-31, keep low to save battery

    private static final int LED_COLOR_INTAKING = Color.rgb(255, 120, 0);  // orange
    private static final int LED_COLOR_FULL = Color.GREEN;
    private static final int LED_COLOR_REVERSE = Color.RED;
    private static final int LED_COLOR_SHOOTING = Color.MAGENTA;

    // Wait after sending "off" so the I2C write actually goes out before the opmode ends
    private static final long LED_OFF_DELAY_MS = 500;

    private int lastLedColor = -1;  // so we only write to I2C when the color changes
    private boolean ledOk = false;   // false = LED missing/failed, opmode keeps running without it

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
        // LED STICK SETUP
        // =========================

        try {
            led = hardwareMap.get(SparkFunLEDStick.class, "led");
            led.setBrightness(LED_BRIGHTNESS);
            led.setColor(Color.BLUE);
            ledOk = true;
        } catch (Exception e) {
            ledOk = false;
            telemetry.addLine("LED stick NOT found - check config name 'led'");
        }

        // =========================
        // HOOD INITIAL POSITION
        // =========================

        hood.setPosition(hoodPosition);

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
        // BATTERY VOLTAGE SENSOR
        // =========================
        // (intake-full threshold is computed each loop in updateBattery())

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        updateBattery();

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
            ledOff();
            sleep(250);
            limelight.stop();
            return;
        }

        imu.resetYaw();

        // =========================
        // MAIN LOOP
        // =========================

        try {

            while (opModeIsActive()) {

                updateBattery();

                updateLimelight();

                hoodControl();

                intake2();

                shooter2();

                updateLeds();

                drivetrain();

                telemetryUpdate();
            }

        } finally {
            limelight.stop();
            ledOff();
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);

            shooter.setVelocity(0);
            intake.setVelocity(0);
            transfur.setVelocity(0);
        }
    }

    // =========================
    // HOOD CONTROL
    // =========================

    private void hoodControl() {

        boolean hoodUp = gamepad1.dpad_right;
        boolean hoodDown = gamepad1.dpad_left;
        boolean cross = gamepad1.cross;
        boolean triangle = gamepad1.triangle;

        // Manual fine adjustment: raise hood
        if (hoodUp && !previousHoodUp) {
            hoodPosition += HOOD_STEP;
        }

        // Manual fine adjustment: lower hood
        if (hoodDown && !previousHoodDown) {
            hoodPosition -= HOOD_STEP;
        }

        // Preset: Cross = 0.25
        if (cross && !previousCross) {
            hoodPosition = HOOD_PRESET_LOW;
        }

        // Preset: Triangle = 0.35
        if (triangle && !previousTriangle) {
            hoodPosition = HOOD_PRESET_HIGH;
        }

        previousHoodUp = hoodUp;
        previousHoodDown = hoodDown;
        previousCross = cross;
        previousTriangle = triangle;

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
            limelightTy = result.getTy();
            limelightHasTarget = true;

            updateDistance(limelightTy);

        } else {

            limelightTx = 0;
            limelightTy = 0;
            limelightHasTarget = false;

            // No target -> no valid distance. Reset filter so the next
            // reading doesn't blend with a stale value.
            hasDistance = false;
            distanceInches = 0;
            filteredDistanceInches = 0;
        }
    }

    // =========================
    // LIMELIGHT DISTANCE
    // =========================
    // distance = (targetHeight - cameraHeight) / tan(cameraMountAngle + ty)

    private void updateDistance(double ty) {

        double totalAngleRad = Math.toRadians(CAMERA_MOUNT_ANGLE_DEG + ty);
        double tanAngle = Math.tan(totalAngleRad);

        // Guard against divide-by-zero / negative angles (bad geometry or bad reading)
        if (tanAngle <= 0.01) {
            hasDistance = false;
            return;
        }

        distanceInches =
                (TARGET_HEIGHT_IN - CAMERA_HEIGHT_IN) / tanAngle
                        + CAMERA_FORWARD_OFFSET_IN;

        if (!hasDistance) {
            // First valid reading: start the filter here
            filteredDistanceInches = distanceInches;
        } else {
            filteredDistanceInches =
                    DISTANCE_FILTER_ALPHA * distanceInches
                            + (1.0 - DISTANCE_FILTER_ALPHA) * filteredDistanceInches;
        }

        hasDistance = true;
    }

    // =========================
    // HORIZONTAL AUTO AIM
    // =========================

    private void auto_aim() {

        if (gamepad1.left_bumper && limelightHasTarget) {

            IsAutoAming = true;
            double error = limelightTx;
            long nowNs = System.nanoTime();

            if (Math.abs(error) <= AIM_TOLERANCE_DEG) {
                aimIntegral = 0;
                AutoPower = 0;
            } else {
                double dt = previousAimTimeNs == 0
                        ? 0.02
                        : (nowNs - previousAimTimeNs) / 1e9;
                dt = Math.max(0.005, Math.min(0.1, dt));

                aimIntegral += error * dt;
                aimIntegral = Math.max(-AIM_INTEGRAL_LIMIT,
                        Math.min(AIM_INTEGRAL_LIMIT, aimIntegral));

                double derivative = previousAimTimeNs == 0
                        ? 0
                        : (error - previousAimError) / dt;
                double output = AIM_P * error
                        + AIM_I * aimIntegral
                        + AIM_D * derivative
                        + AIM_F * Math.signum(error);

                output = Math.max(-AIM_MAX_POWER,
                        Math.min(AIM_MAX_POWER, output));
                if (Math.abs(output) < AIM_MIN_POWER) {
                    output = AIM_MIN_POWER * Math.signum(error);
                }
                AutoPower = output;
            }

            previousAimError = error;
            previousAimTimeNs = nowNs;
            telemetry.addData("Auto Aim", "PIDF Active");
            telemetry.addData("Auto Power", AutoPower);
            telemetry.addData("TX Error", error);

        } else {
            IsAutoAming = false;
            AutoPower = 0;
            aimIntegral = 0;
            previousAimError = 0;
            previousAimTimeNs = 0;
            telemetry.addData("Auto Aim", gamepad1.left_bumper ? "No Target" : "Off");
        }
    }

    // =========================
    // BATTERY VOLTAGE COMPENSATION
    // =========================

    private void updateBattery() {

        double v = voltageSensor.getVoltage();

        // Ignore bad readings, keep the last good value
        if (v > 1.0) {
            batteryVoltage = v;
        }

        double clamped = Math.max(
                MIN_VOLTAGE_CLAMP,
                Math.min(MAX_VOLTAGE_CLAMP, batteryVoltage)
        );

        intakeStopAmps =
                INTAKE_STOP_AMPS_AT_NOMINAL
                        * Math.pow(NOMINAL_VOLTAGE / clamped, VOLTAGE_COMP_GAIN);
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

        } else if (intake.getCurrent(CurrentUnit.AMPS) > intakeStopAmps) {
            my_4_balls = true;
        }

        if (my_4_balls) {
            intake.setVelocity(0);

        } else {
            intake.setVelocity(2400);
        }
    }

    // =========================
    // LED STATUS
    // =========================

    private void updateLeds() {

        int color;

        if (gamepad1.share) {
            color = LED_COLOR_REVERSE;      // spitting balls out

        } else if (my_4_balls) {
            color = LED_COLOR_FULL;         // intake full / locked out

        } else if (gamepad1.right_bumper) {
            color = LED_COLOR_SHOOTING;     // feeding the shooter

        } else {
            color = LED_COLOR_INTAKING;     // intaking, not full yet
        }

        // Only talk to the LED stick when something actually changed
        if (ledOk && color != lastLedColor) {
            try {
                led.setColor(color);
                lastLedColor = color;
            } catch (Exception e) {
                ledOk = false;  // give up on the LED, never crash the opmode
            }
        }
    }

    private void ledOff() {
        if (!ledOk) return;
        try {
            led.turnAllOff();

            // Wait for the I2C write to go out. Uses a timer instead of sleep()
            // because sleep() can return instantly once a stop has been requested.
            long end = System.nanoTime() + LED_OFF_DELAY_MS * 1_000_000L;
            while (System.nanoTime() < end) {
                // wait
            }

            // Send it again in case the first write was dropped
            led.turnAllOff();
        } catch (Exception ignored) {
        }
    }

    // =========================
    // SHOOTER AND TRANSFER
    // =========================

    // Spin-up timer: runs while the shooter is not ready, stops when it becomes ready
    private void updateSpinUpTimer(boolean shooterReady) {

        // Shooter off -> nothing to time
        if (flyspeed <= 0) {
            spinUpTiming = false;
            spinUpSeconds = 0;
            return;
        }

        if (!shooterReady) {

            if (!spinUpTiming) {
                spinUpTiming = true;
                spinUpStartNs = System.nanoTime();
            }

            spinUpSeconds = (System.nanoTime() - spinUpStartNs) / 1e9;

        } else if (spinUpTiming) {

            spinUpTiming = false;
            spinUpSeconds = (System.nanoTime() - spinUpStartNs) / 1e9;
            lastSpinUpSeconds = spinUpSeconds;
        }
    }

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

        double actualShooterVelocity = shooter.getVelocity();

        shooter.setVelocity(flyspeed);

        boolean shooterReady = actualShooterVelocity >= flyspeed - 100;

        updateSpinUpTimer(shooterReady);

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
            rotate = AutoPower;
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
                "Spin-Up Timer",
                spinUpTiming
                        ? String.format("%.2f s (spinning up...)", spinUpSeconds)
                        : "READY");

        telemetry.addData("Last Spin-Up (s)", "%.2f", lastSpinUpSeconds);

        telemetry.addData(
                "Shooter Current",
                shooter.getCurrent(CurrentUnit.AMPS)
        );

        telemetry.addLine("================");

        telemetry.addLine("=== LIMELIGHT ===");

        telemetry.addData("Auto Aiming", IsAutoAming);
        telemetry.addData("Limelight Target", limelightHasTarget);
        telemetry.addData("TX", limelightTx);
        telemetry.addData("TY", limelightTy);
        telemetry.addData("Auto Power", AutoPower);

        telemetry.addLine("--- DISTANCE ---");

        if (hasDistance) {
            telemetry.addData("Distance (in)", "%.1f", filteredDistanceInches);
            telemetry.addData("Distance (cm)", "%.1f", filteredDistanceInches * 2.54);
            telemetry.addData("Distance (ft)", "%.2f", filteredDistanceInches / 12.0);
            telemetry.addData("Distance Raw (in)", "%.1f", distanceInches);
        } else {
            telemetry.addData("Distance", "NO TARGET");
        }

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
        telemetry.addData("Battery (V)", "%.2f", batteryVoltage);
        telemetry.addData("Intake Stop Threshold (A)", "%.2f", intakeStopAmps);

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
