package org.firstinspires.ftc.teamcode.Commands.AutoCommands;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "twotip_park", group = "Autonomous")
public class twotip_park extends OpMode {

    // =========================
    // PATHING
    // =========================
    public Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private int pathState;
    private ElapsedTime pathTimer;
    private Paths paths;

    // =========================
    // HARDWARE (same names as teleop)
    // =========================
    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx intake, transfur, shooter;
    private Limelight3A limelight;

    // =========================
    // FLYWHEEL (copied from teleop)
    // =========================
    private double flyspeed = 1550;

    private static final double BB_FULL_POWER = 1.0;
    private static final double BB_HOLD_POWER = 0.0;
    private static final double VEL_ENTER_BAND = 60;
    private static final double VEL_EXIT_BAND = 120;

    private static final double SHOOTER_P = 1.17;
    private static final double SHOOTER_I = 0.117;
    private static final double SHOOTER_D = 0;
    private static final double SHOOTER_F = 11.7;

    private boolean inVelocityControl = false;
    private boolean shooterEncoderMode = true;
    private boolean shooterEnabled = false;

    // =========================
    // AUTO AIM (copied from teleop)
    // =========================
    private static final double AIM_DEADBAND = 1.25;     // degrees
    private static final double AIM_SETTLE_TIME = 0.20;  // seconds inside deadband before shooting
    private static final double AIM_TIMEOUT = 2.0;       // give up aiming and shoot anyway
    private static final double SHOOT_TIME = 5.0;        // how long to feed balls

    private boolean limelightHasTarget = false;
    private double limelightTx = 0;
    private ElapsedTime aimSettleTimer = new ElapsedTime();

    // =========================
    // STATES
    // =========================
    private static final int DRIVE_TO_SHOOT = 0;
    private static final int WAIT_FOR_PATH = 1;
    private static final int AIM = 2;
    private static final int SHOOT = 3;
    private static final int DONE = 4;

    @Override
    public void init() {
        // ----- Pedro -----
        follower = Constants.create(hardwareMap);
        follower.setPose(p.of(9.000, 25.000, 0.000));
        pathTimer = new ElapsedTime();
        paths = new Paths(follower);

        // ----- Hardware -----
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfur = hardwareMap.get(DcMotorEx.class, "transfur");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // Same directions as teleop (needed for direct motor control while aiming)
        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        shooter.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterEncoderMode = true;
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfur.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfur.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ----- Limelight -----
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
        shooterEnabled = true;          // spin up flywheel while driving
        setPathState(DRIVE_TO_SHOOT);
    }

    @Override
    public void loop() {
        // Only let Pedro drive while it is actually following the path.
        // After that we drive the motors directly for aiming.
        if (pathState <= WAIT_FOR_PATH) {
            follower.update();
        }

        updateLimelight();
        updateShooter();
        pathState = autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", follower.pose().heading());
        telemetry.addData("Limelight Target", limelightHasTarget);
        telemetry.addData("TX", limelightTx);
        telemetry.addData("Shooter Target", flyspeed);
        telemetry.addData("Shooter Actual", shooter.getVelocity());
        telemetry.addData("Shooter Ready", shooterReady());
        telemetry.update();
    }

    @Override
    public void stop() {
        setDrivePower(0, 0, 0);
        shooterEnabled = false;
        setShooterEncoderMode(true);
        shooter.setVelocity(0);
        intake.setVelocity(0);
        transfur.setVelocity(0);
        limelight.stop();
    }

    // =========================
    // PATHS
    // =========================
    public static class Paths {

        private static final PoseFactory p = PoseFactory.degrees();
        public Path DriveToShoot;

        public Paths(Follower follower) {
            // NOTE: check this line, see my earlier note about the heading/units of linear().
            DriveToShoot = line(
                    p.of(9.000, 25.000, 0.0),
                    p.of(60.000, 25.000, 0.0)
            ).linear(Math.toRadians(0), Math.toRadians(90));
        }
    }

    // =========================
    // STATE MACHINE
    // =========================
    public int autonomousPathUpdate() {
        switch (pathState) {

            case DRIVE_TO_SHOOT:
                follower.follow(paths.DriveToShoot);
                setPathState(WAIT_FOR_PATH);
                break;

            case WAIT_FOR_PATH:
                if (!follower.isBusy()) {
                    aimSettleTimer.reset();
                    setPathState(AIM);
                }
                break;

            case AIM:
                boolean aimed = false;

                if (limelightHasTarget) {
                    if (Math.abs(limelightTx) <= AIM_DEADBAND) {
                        setDrivePower(0, 0, 0);
                        // must stay centered for a moment so we don't shoot mid-swing
                        aimed = aimSettleTimer.seconds() >= AIM_SETTLE_TIME;
                    } else {
                        aimSettleTimer.reset();
                        double power = Math.abs(limelightTx) / 42.5;
                        if (power < 0.175) power = 0.155;   // same minimum as teleop
                        setDrivePower(0, 0, limelightTx > 0 ? power : -power);
                    }
                } else {
                    setDrivePower(0, 0, 0);   // no target: don't spin blindly
                }

                // Shoot when aimed, or after the timeout so we never get stuck
                if (aimed || pathTimer.seconds() > AIM_TIMEOUT) {
                    setDrivePower(0, 0, 0);
                    setPathState(SHOOT);
                }
                break;

            case SHOOT:
                setDrivePower(0, 0, 0);
                intake.setVelocity(2400);

                // Feed only while the flywheel is up to speed (same as teleop)
                transfur.setVelocity(shooterReady() ? 2600 : 0);

                if (pathTimer.seconds() > SHOOT_TIME) {
                    setPathState(DONE);
                }
                break;

            case DONE:
                shooterEnabled = false;
                intake.setVelocity(0);
                transfur.setVelocity(0);
                requestOpModeStop();
                pathState = -1;
                break;
        }
        return pathState;
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

    // =========================
    // LIMELIGHT
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
    // SHOOTER (hybrid bang-bang + velocity PIDF, from teleop)
    // =========================
    private boolean shooterReady() {
        return shooterEnabled && shooter.getVelocity() >= flyspeed - 100;
    }

    private void setShooterEncoderMode(boolean useEncoder) {
        if (useEncoder == shooterEncoderMode) return;
        shooter.setMode(useEncoder
                ? DcMotor.RunMode.RUN_USING_ENCODER
                : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterEncoderMode = useEncoder;
    }

    private void updateShooter() {
        if (!shooterEnabled) {
            inVelocityControl = false;
            setShooterEncoderMode(false);
            shooter.setPower(0);
            return;
        }

        double error = flyspeed - shooter.getVelocity();

        if (!inVelocityControl && Math.abs(error) <= VEL_ENTER_BAND) {
            inVelocityControl = true;
        } else if (inVelocityControl && Math.abs(error) > VEL_EXIT_BAND) {
            inVelocityControl = false;
        }

        if (inVelocityControl) {
            setShooterEncoderMode(true);
            shooter.setVelocity(flyspeed);
        } else {
            setShooterEncoderMode(false);
            shooter.setPower(error > 0 ? BB_FULL_POWER : BB_HOLD_POWER);
        }
    }

    // =========================
    // DIRECT MECANUM DRIVE (same math as teleop)
    // =========================
    private void setDrivePower(double forward, double right, double rotate) {
        double flP = forward + right + rotate;
        double frP = forward - right - rotate;
        double brP = forward + right - rotate;
        double blP = forward - right + rotate;

        double max = Math.max(1.0,
                Math.max(Math.max(Math.abs(flP), Math.abs(frP)),
                        Math.max(Math.abs(blP), Math.abs(brP))));

        fl.setPower(flP / max);
        fr.setPower(frP / max);
        bl.setPower(blP / max);
        br.setPower(brP / max);
    }
}