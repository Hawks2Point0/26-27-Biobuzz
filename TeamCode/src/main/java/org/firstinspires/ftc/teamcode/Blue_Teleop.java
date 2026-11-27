package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
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

    private DcMotorEx br, fr, fl, bl;
    private DcMotorEx intake;
    private DcMotorEx transfur;
    private DcMotorEx shooter;

    private Servo hood;

    private Limelight3A limelight;
    private IMU imu;

    // =========================
    // PER-LOOP CACHED READS (read hardware once per loop)
    // =========================

    private double yawDeg = 0;
    private double shooterVel = 0;
    private double intakeAmps = 0;

    // =========================
    // HOOD SERVO
    // =========================

    private double hoodPosition = 0.26;

    private static final double HOOD_STEP = 0.025;
    private static final double HOOD_MIN = 0.0;
    private static final double HOOD_MAX = 0.5;

    private static final double HOOD_PRESET_LOW = 0.25;   // Cross
    private static final double HOOD_PRESET_HIGH = 0.35;  // Triangle

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
    private static final double FLYSPEED_PRESET = 1550;   // Square

    private boolean previousUp = false;
    private boolean previousDown = false;
    private boolean previousSquare = false;

    // =========================
    // SHOOTER PIDF
    // =========================

    private static final double SHOOTER_P = 81.8;
    private static final double SHOOTER_I = 0.2;
    private static final double SHOOTER_D = 0;
    private static final double SHOOTER_F = 11.9588;

    // =========================
    // SHOOTER READY (band + debounce)
    // =========================

    private static final double READY_TOLERANCE = 75;              // ticks/sec either side of target
    private static final long READY_HOLD_NS = 60_000_000L;         // must stay in band 60 ms
    private long readySinceNs = 0;

    // =========================
    // TRANSFER / INTAKE SPEEDS
    // =========================

    private static final double TRANSFER_VEL = 2600;
    private static final double INTAKE_VEL = 2400;
    private static final double INTAKE_REVERSE_VEL = -2450;

    // =========================
    // SPIN-UP TIMER
    // =========================

    private boolean spinUpTiming = false;
    private long spinUpStartNs = 0;
    private double spinUpSeconds = 0;
    private double lastSpinUpSeconds = 0;

    // =========================
    // LIMELIGHT
    // =========================

    // Blue goal AprilTag ID (DECODE). Set to -1 to accept any tag. VERIFY on your field/pipeline.
    private static final int TARGET_TAG_ID = 20;
    private static final long MAX_STALENESS_MS = 100;

    private boolean limelightHasTarget = false;
    private boolean newLlFrame = false;
    private double lastLlTimestamp = -1;

    private double limelightTx = 0;
    private double limelightTy = 0;

    // =========================
    // AUTO AIM (IMU fast loop, Limelight sets the target heading)
    // =========================

    private boolean isAutoAiming = false;
    private boolean aimed = false;
    private double targetYawDeg = 0;
    private double autoPower = 0;
    private double aimErrorDeg = 0;

    private static final double AIM_KP = 0.02;
    private static final double AIM_KD = 0.0015;
    private static final double AIM_KS = 0.05;          // small friction boost, only outside deadband
    private static final double AIM_ENTER_DEG = 1.0;    // considered aimed
    private static final double AIM_EXIT_DEG = 2.0;     // start moving again (hysteresis)
    private static final double AIM_MAX = 0.5;

    // =========================
    // LIMELIGHT DISTANCE
    // =========================
    // TUNE: inches / degrees
    private static final double CAMERA_HEIGHT_IN = 10.0;
    private static final double TARGET_HEIGHT_IN = 29.5;
    private static final double CAMERA_MOUNT_ANGLE_DEG = 45.0;
    private static final double CAMERA_FORWARD_OFFSET_IN = 0.0;
    private static final double DISTANCE_FILTER_ALPHA = 0.3;

    private double distanceInches = 0;
    private double filteredDistanceInches = 0;
    private boolean hasDistance = false;

    // =========================
    // INTAKE PROTECTION
    // =========================

    private boolean intakeFull = false;

    private static final double INTAKE_STOP_AMPS_AT_NOMINAL = 3.425;
    private static final double VOLTAGE_COMP_GAIN = 1.0;
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double MIN_VOLTAGE_CLAMP = 9.0;
    private static final double MAX_VOLTAGE_CLAMP = 14.0;

    // Ignore inrush current after the intake (re)starts, and require sustained over-current
    private static final long INTAKE_STARTUP_IGNORE_MS = 400;
    private static final long INTAKE_FULL_DEBOUNCE_MS = 250;

    private final ElapsedTime intakeRunTimer = new ElapsedTime();
    private final ElapsedTime overCurrentTimer = new ElapsedTime();
    private boolean overCurrentActive = false;
    private boolean wasIntakeRunning = false;

    private VoltageSensor voltageSensor;
    private double batteryVoltage = NOMINAL_VOLTAGE;
    private double intakeStopAmps = INTAKE_STOP_AMPS_AT_NOMINAL;

    // =========================
    // LED STICK (SparkFun Qwiic)
    // =========================

    private SparkFunLEDStick led;

    private static final int LED_BRIGHTNESS = 5;

    private static final int LED_COLOR_INTAKING = Color.rgb(255, 120, 0);
    private static final int LED_COLOR_FULL = Color.GREEN;
    private static final int LED_COLOR_REVERSE = Color.RED;
    private static final int LED_COLOR_SHOOTING = Color.MAGENTA;

    private static final long LED_OFF_DELAY_MS = 500;

    private int lastLedColor = -1;
    private int lastPadColor = -1;   // gamepad light bar, only written on change
    private boolean ledOk = false;

    // =========================
    // LOOP TIMING
    // =========================

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopMs = 0;

    @Override
    public void runOpMode() {

        // =========================
        // BULK CACHING
        // =========================

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

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

        gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);

        hood.setPosition(hoodPosition);

        // =========================
        // LIMELIGHT SETUP
        // =========================

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        // =========================
        // DRIVETRAIN PIDF
        // =========================

        br.setVelocityPIDFCoefficients(1.0708, 0.1071, 0, 10.7087);
        fr.setVelocityPIDFCoefficients(1.0435, 0.1044, 0, 10.4254);
        fl.setVelocityPIDFCoefficients(1.785, 0.1078, 0, 10.70882);
        bl.setVelocityPIDFCoefficients(0.9999, 0.0999, 0, 9.999);

        // =========================
        // FLYWHEEL PIDF
        // =========================

        shooter.setVelocityPIDFCoefficients(
                SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);

        // =========================
        // MOTOR DIRECTIONS / MODES
        // =========================

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

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

        transfur.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // =========================
        // BATTERY VOLTAGE SENSOR
        // =========================

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
            limelight.stop();
            ledOff();
            return;
        }

        imu.resetYaw();
        loopTimer.reset();

        // =========================
        // MAIN LOOP
        // =========================

        try {

            while (opModeIsActive()) {

                loopMs = loopTimer.milliseconds();
                loopTimer.reset();

                // Read hardware once per loop
                yawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                shooterVel = shooter.getVelocity();
                intakeAmps = intake.getCurrent(CurrentUnit.AMPS);

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
            // Stop motors FIRST, then slow stuff (LED has a blocking delay)
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);

            shooter.setVelocity(0);
            intake.setVelocity(0);
            transfur.setVelocity(0);

            try {
                gamepad1.setLedColor(0, 0, 0, 250);
            } catch (Exception ignored) {
            }

            limelight.stop();
            ledOff();
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

        if (hoodUp && !previousHoodUp) hoodPosition += HOOD_STEP;
        if (hoodDown && !previousHoodDown) hoodPosition -= HOOD_STEP;
        if (cross && !previousCross) hoodPosition = HOOD_PRESET_LOW;
        if (triangle && !previousTriangle) hoodPosition = HOOD_PRESET_HIGH;

        previousHoodUp = hoodUp;
        previousHoodDown = hoodDown;
        previousCross = cross;
        previousTriangle = triangle;

        hoodPosition = Math.max(HOOD_MIN, Math.min(HOOD_MAX, hoodPosition));
        hood.setPosition(hoodPosition);
    }

    // =========================
    // LIMELIGHT UPDATE
    // =========================

    private void updateLimelight() {

        LLResult result = limelight.getLatestResult();

        boolean good = false;
        double tx = 0, ty = 0;

        if (result != null && result.isValid() && result.getStaleness() < MAX_STALENESS_MS) {

            for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
                if (TARGET_TAG_ID < 0 || f.getFiducialId() == TARGET_TAG_ID) {
                    tx = f.getTargetXDegrees();
                    ty = f.getTargetYDegrees();
                    good = true;
                    break;
                }
            }
        }

        if (good) {

            newLlFrame = result.getTimestamp() != lastLlTimestamp;
            lastLlTimestamp = result.getTimestamp();

            limelightTx = tx;
            limelightTy = ty;
            limelightHasTarget = true;

            if (newLlFrame) updateDistance(ty);

        } else {

            newLlFrame = false;
            limelightTx = 0;
            limelightTy = 0;
            limelightHasTarget = false;

            hasDistance = false;
            distanceInches = 0;
            filteredDistanceInches = 0;
        }
    }

    // distance = (targetHeight - cameraHeight) / tan(cameraMountAngle + ty)
    private void updateDistance(double ty) {

        double tanAngle = Math.tan(Math.toRadians(CAMERA_MOUNT_ANGLE_DEG + ty));

        if (tanAngle <= 0.01) {
            hasDistance = false;
            return;
        }

        distanceInches =
                (TARGET_HEIGHT_IN - CAMERA_HEIGHT_IN) / tanAngle
                        + CAMERA_FORWARD_OFFSET_IN;

        if (!hasDistance) {
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
    // Limelight tx -> target heading (only on NEW frames).
    // PD runs on IMU heading error every loop (no camera latency in the fast loop).
    //
    // Sign convention: IMU yaw is CCW-positive. Positive tx = target to the right,
    // so target heading = yaw - tx. error > 0 means "turn right" (positive rotate).

    private void autoAim() {

        if (gamepad1.left_bumper && limelightHasTarget) {

            // Fresh engage or new frame: refresh the target heading
            if (!isAutoAiming || newLlFrame) {
                targetYawDeg = yawDeg - limelightTx;
            }
            if (!isAutoAiming) aimed = false;
            isAutoAiming = true;

            double error = AngleUnit.DEGREES.normalize(yawDeg - targetYawDeg);
            // + when the robot is turning right
            double turnRateRight =
                    -imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;

            aimErrorDeg = error;

            if (Math.abs(error) < AIM_ENTER_DEG) {
                aimed = true;
            } else if (Math.abs(error) > AIM_EXIT_DEG) {
                aimed = false;
            }

            if (aimed) {
                autoPower = 0;
            } else {
                double out = AIM_KP * error
                        - AIM_KD * turnRateRight
                        + AIM_KS * Math.signum(error);
                autoPower = Math.max(-AIM_MAX, Math.min(AIM_MAX, out));
            }

        } else {
            isAutoAiming = false;
            aimed = false;
            autoPower = 0;
            aimErrorDeg = 0;
        }
    }

    // =========================
    // BATTERY VOLTAGE COMPENSATION
    // =========================

    private void updateBattery() {

        double v = voltageSensor.getVoltage();

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
            intakeFull = false;
        }

        if (gamepad1.share) {
            intake.setVelocity(INTAKE_REVERSE_VEL);
            wasIntakeRunning = false;
            overCurrentActive = false;
            return;
        }

        if (gamepad1.right_bumper) {
            intakeFull = false;
            overCurrentActive = false;

        } else if (!intakeFull) {

            boolean over = intakeAmps > intakeStopAmps
                    && intakeRunTimer.milliseconds() > INTAKE_STARTUP_IGNORE_MS;

            if (over) {
                if (!overCurrentActive) {
                    overCurrentActive = true;
                    overCurrentTimer.reset();
                } else if (overCurrentTimer.milliseconds() > INTAKE_FULL_DEBOUNCE_MS) {
                    intakeFull = true;
                }
            } else {
                overCurrentActive = false;
            }
        }

        boolean running = !intakeFull;

        // Intake just (re)started -> ignore inrush current for a moment
        if (running && !wasIntakeRunning) {
            intakeRunTimer.reset();
        }
        wasIntakeRunning = running;

        intake.setVelocity(running ? INTAKE_VEL : 0);
    }

    // =========================
    // LED STATUS
    // =========================

    private void updateLeds() {

        int color;

        if (gamepad1.share) {
            color = LED_COLOR_REVERSE;
        } else if (intakeFull) {
            color = LED_COLOR_FULL;
        } else if (gamepad1.right_bumper) {
            color = LED_COLOR_SHOOTING;
        } else {
            color = LED_COLOR_INTAKING;
        }

        // Controller light bar (works even if the LED stick is missing)
        if (color != lastPadColor) {
            gamepad1.setLedColor(
                    Color.red(color) / 255.0,
                    Color.green(color) / 255.0,
                    Color.blue(color) / 255.0,
                    Gamepad.LED_DURATION_CONTINUOUS);
            lastPadColor = color;
        }

        // LED stick: only talk to it when something actually changed
        if (ledOk && color != lastLedColor) {
            try {
                led.setColor(color);
                lastLedColor = color;
            } catch (Exception e) {
                ledOk = false;
            }
        }
    }

    private void ledOff() {
        if (!ledOk) return;
        try {
            led.turnAllOff();

            long end = System.nanoTime() + LED_OFF_DELAY_MS * 1_000_000L;
            while (System.nanoTime() < end) {
                // wait
            }

            led.turnAllOff();
        } catch (Exception ignored) {
        }
    }

    // =========================
    // SHOOTER AND TRANSFER
    // =========================

    private boolean isShooterReady() {

        boolean inBand = flyspeed > 0
                && Math.abs(shooterVel - flyspeed) <= READY_TOLERANCE;

        if (!inBand) {
            readySinceNs = 0;
            return false;
        }

        if (readySinceNs == 0) readySinceNs = System.nanoTime();

        return System.nanoTime() - readySinceNs >= READY_HOLD_NS;
    }

    private void updateSpinUpTimer(boolean shooterReady) {

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
        boolean square = gamepad1.square;

        if (up && !previousUp) flyspeed += SPEED_STEP;
        if (down && !previousDown) flyspeed -= SPEED_STEP;
        if (square && !previousSquare) flyspeed = FLYSPEED_PRESET;

        previousUp = up;
        previousDown = down;
        previousSquare = square;

        flyspeed = Math.max(MIN_FLYSPEED, Math.min(MAX_FLYSPEED, flyspeed));

        shooter.setVelocity(flyspeed);

        boolean shooterReady = isShooterReady();

        updateSpinUpTimer(shooterReady);

        if (gamepad1.right_bumper && shooterReady) {
            transfur.setVelocity(TRANSFER_VEL);
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

        autoAim();

        if (isAutoAiming && limelightHasTarget) {
            // Auto-aim gets full drivetrain priority
            forward = 0;
            right = 0;
            rotate = autoPower;
        }

        if (gamepad1.options) {
            drive(forward, right, rotate);                // robot-centric
        } else {
            driveFieldCentric(forward, right, rotate);
        }
    }

    // =========================
    // FIELD CENTRIC
    // =========================

    private void driveFieldCentric(double forward, double right, double rotate) {

        double yawRad = Math.toRadians(yawDeg);
        double cos = Math.cos(yawRad);
        double sin = Math.sin(yawRad);

        // Rotate the stick vector by -yaw into the robot frame
        double rightRobot = right * cos + forward * sin;
        double forwardRobot = forward * cos - right * sin;

        drive(forwardRobot, rightRobot, rotate);
    }

    // =========================
    // MECANUM DRIVE
    // =========================

    private void drive(double forward, double right, double rotate) {

        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
                )
        );

        fl.setPower(frontLeftPower / maxPower);
        fr.setPower(frontRightPower / maxPower);
        bl.setPower(backLeftPower / maxPower);
        br.setPower(backRightPower / maxPower);
    }

    // =========================
    // TELEMETRY
    // =========================

    private void telemetryUpdate() {

        telemetry.addData("Loop (ms)", "%.1f", loopMs);

        telemetry.addLine("=== FLYWHEEL ===");
        telemetry.addData("Target Speed", flyspeed);
        telemetry.addData("Actual Speed", shooterVel);
        telemetry.addData("Shooter Power", shooter.getPower());
        telemetry.addData("Shooter Ready", readySinceNs != 0
                && System.nanoTime() - readySinceNs >= READY_HOLD_NS ? "YES" : "WAITING");
        telemetry.addData("Speed Difference", flyspeed - shooterVel);
        telemetry.addData("Spin-Up Timer",
                spinUpTiming
                        ? String.format("%.2f s (spinning up...)", spinUpSeconds)
                        : "READY");
        telemetry.addData("Last Spin-Up (s)", "%.2f", lastSpinUpSeconds);
        telemetry.addData("Shooter Current", shooter.getCurrent(CurrentUnit.AMPS));

        telemetry.addLine("=== LIMELIGHT / AIM ===");
        telemetry.addData("Auto Aiming", isAutoAiming);
        telemetry.addData("Aimed", aimed);
        telemetry.addData("Limelight Target", limelightHasTarget);
        telemetry.addData("TX", "%.2f", limelightTx);
        telemetry.addData("TY", "%.2f", limelightTy);
        telemetry.addData("Aim Error (deg)", "%.2f", aimErrorDeg);
        telemetry.addData("Target Yaw", "%.2f", targetYawDeg);
        telemetry.addData("Auto Power", "%.3f", autoPower);

        if (hasDistance) {
            telemetry.addData("Distance (in)", "%.1f", filteredDistanceInches);
            telemetry.addData("Distance (ft)", "%.2f", filteredDistanceInches / 12.0);
            telemetry.addData("Distance Raw (in)", "%.1f", distanceInches);
        } else {
            telemetry.addData("Distance", "NO TARGET");
        }

        telemetry.addLine("=== DRIVE / INTAKE ===");
        telemetry.addData("FR Velocity", fr.getVelocity());
        telemetry.addData("FL Velocity", fl.getVelocity());
        telemetry.addData("BR Velocity", br.getVelocity());
        telemetry.addData("BL Velocity", bl.getVelocity());

        telemetry.addData("Intake Velocity", intake.getVelocity());
        telemetry.addData("Intake Amps", "%.2f", intakeAmps);
        telemetry.addData("Intake Full", intakeFull);
        telemetry.addData("Battery (V)", "%.2f", batteryVoltage);
        telemetry.addData("Intake Stop Threshold (A)", "%.2f", intakeStopAmps);

        telemetry.addData("Transfer Velocity", transfur.getVelocity());
        telemetry.addData("Hood Position", "%.3f", hoodPosition);
        telemetry.addData("Yaw", "%.1f", yawDeg);

        telemetry.update();
    }
}