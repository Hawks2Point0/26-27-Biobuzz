/* ============================================================= *
 *                 Turtle Tracer — Auto-Generated                *
 *                                                               *
 *  Version: 2.7.0.                                              *
 *  Copyright (c) 2026 Matthew Allen                             *
 *                                                               *
 *  THIS FILE IS AUTO-GENERATED — DO NOT EDIT MANUALLY.          *
 *  Changes will be overwritten when regenerated.                *
 * ============================================================= */

package org.firstinspires.ftc.teamcode.Commands.AutoCommands;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.turtletracerlib.auto.Alliance;
import com.turtletracerlib.auto.AutoConfig;
import com.turtletracerlib.auto.AutoContext;
import com.turtletracerlib.auto.AutoRegistry;
import com.turtletracerlib.auto.AutoSelector;
import com.turtletracerlib.auto.MirrorAxes;
import com.turtletracerlib.auto.MirrorPolicy;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "shoot_park", group = "Autonomous")
public class shoot_park extends OpMode {

    // ===== Shooting settings (values taken from Blue_Teleop) =====
    private static final double SHOOTER_P = 61.8;
    private static final double SHOOTER_I = 0.1;
    private static final double SHOOTER_D = 0.1460;
    private static final double SHOOTER_F = 11.4608;

    private static final double FLY_SPEED = 1650; // Shooter target velocity
    private static final double READY_TOLERANCE = 100; // Ticks/sec below target still counted as ready
    private static final double HOOD_POSITION = 0.26;
    private static final double INTAKE_VELOCITY = 2400;
    private static final double TRANSFER_VELOCITY = 2600;

    private static final double SPINUP_TIMEOUT_MS = 2000; // Shoot anyway if the flywheel never reaches speed
    private static final double SHOOT_TIME_MS = 3000; // How long to feed balls into the shooter

    private DcMotorEx shooter;
    private DcMotorEx intake;
    private DcMotorEx transfur;
    private Servo hood;

    public Follower follower; // Pathing follower instance
    private final PoseFactory p = PoseFactory.degrees();
    private int pathState; // Current autonomous path state (state machine)
    private ElapsedTime pathTimer; // Timer for path state machine
    private Paths paths; // Paths defined in the Paths class
    private AutoSelector<AutoContext> selector; // Alliance menu shown during init
    private AutoContext ctx; // The alliance picked, to mirror the paths for

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        follower = Constants.create(hardwareMap);

        // Shooting hardware
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfur = hardwareMap.get(DcMotorEx.class, "transfur");
        hood = hardwareMap.get(Servo.class, "hood");

        intake.setDirection(DcMotor.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfur.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);

        transfur.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hood.setPosition(HOOD_POSITION);

        pathTimer = new ElapsedTime();
        // Alliance menu: D-pad left/right picks the alliance before Start.
        selector = new AutoSelector<>(
                new AutoRegistry<AutoContext>().add("2tip_park", c -> c),
                AutoConfig.builder()
                        .baseAlliance(Alliance.RED) // The side these paths are drawn for
                        .mirrorPolicy(MirrorPolicy.FOLLOW_ALLIANCE) // Mirrored when the driver picks the other one
                        .mirrorAxes(MirrorAxes.BOTH)
                        .build()
        );
    }

    @Override
    public void init_loop() {
        selector.update(gamepad1, telemetry); // Draw the alliance menu
    }

    @Override
    public void start() {
        ctx = selector.buildSelected(); // The alliance picked, and whether to mirror
        follower.setPose(ctx.pose(p.of(9.000, 135.000, 0.000)));
        paths = new Paths(follower, ctx); // Build paths for that alliance

        // Spin up the flywheel and intake while driving to the shooting spot
        shooter.setVelocity(FLY_SPEED);
        intake.setVelocity(INTAKE_VELOCITY);
    }

    @Override
    public void loop() {
        follower.update(); // Update follower
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Driver Station
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", follower.pose().heading());
        telemetry.addData("Shooter Target", FLY_SPEED);
        telemetry.addData("Shooter Actual", shooter.getVelocity());
        telemetry.addData("Shooter Ready", shooterReady());
        telemetry.update();
    }

    @Override
    public void stop() {
        stopShooting();
        shooter.setVelocity(0);
        intake.setVelocity(0);
    }

    private boolean shooterReady() {
        return shooter.getVelocity() >= FLY_SPEED - READY_TOLERANCE;
    }

    private void stopShooting() {
        transfur.setVelocity(0);
    }

    public static class Paths {

        private static final PoseFactory p = PoseFactory.degrees();
        public Path DriveToShoot;
        public Path line2;

        public Paths(Follower follower, AutoContext ctx) {
            DriveToShoot = curve(
                    ctx.pose(p.of(9.000, 135.000, 0.0)),
                    ctx.pose(p.of(34.654, 111.635, 0.0)),
                    ctx.pose(p.of(60.000, 118.874, 0.0))
            ).linear(
                    ctx.heading(Math.toRadians(0)),
                    ctx.heading(Math.toRadians(-90))
            );

            line2 = line(
                    ctx.pose(p.of(60.000, 118.874, 0.0)),
                    ctx.pose(p.of(12.085, 111.528, 0.0))
            ).linear(
                    ctx.heading(Math.toRadians(-90)),
                    ctx.heading(Math.toRadians(-90))
            );
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.follow(paths.DriveToShoot);
                setPathState(1);
                break;
            case 1:
                // Arrived at the shooting spot (flywheel has been spinning up the whole drive)
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;
            case 2:
                // Wait for the flywheel to reach speed (or give up after the timeout)
                shooter.setVelocity(FLY_SPEED);
                if (shooterReady() || pathTimer.milliseconds() > SPINUP_TIMEOUT_MS) {
                    setPathState(3);
                }
                break;
            case 3:
                // Feed balls into the shooter
                transfur.setVelocity(TRANSFER_VELOCITY);
                if (pathTimer.milliseconds() > SHOOT_TIME_MS) {
                    stopShooting();
                    setPathState(4);
                }
                break;
            case 4:
                follower.follow(paths.line2);
                setPathState(5);
                break;
            case 5:
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;
            case 6:
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
}
