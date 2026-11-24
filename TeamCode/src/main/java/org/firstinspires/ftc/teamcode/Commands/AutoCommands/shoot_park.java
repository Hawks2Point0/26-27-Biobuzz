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
import static com.pedropathing.api.Paths.path;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.turtletracerlib.auto.Alliance;
import com.turtletracerlib.auto.AutoConfig;
import com.turtletracerlib.auto.AutoContext;
import com.turtletracerlib.auto.AutoRegistry;
import com.turtletracerlib.auto.AutoSelector;
import com.turtletracerlib.auto.MirrorAxes;
import com.turtletracerlib.auto.MirrorPolicy;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfur;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "shoot_park", group = "Autonomous")
public class shoot_park extends OpMode {

    private static final double SHOOT_SPEED = 1550; // same default as teleop

    public Follower follower; // Pathing follower instance
    private final PoseFactory p = PoseFactory.degrees();
    private int pathState; // Current autonomous path state (state machine)
    private ElapsedTime pathTimer; // Timer for path state machine
    private Paths paths; // Paths defined in the Paths class
    private AutoSelector<AutoContext> selector; // Alliance menu shown during init
    private AutoContext ctx; // The alliance picked, to mirror the paths for
    private Shooter shooter;
    private Transfur transfur;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        shooter = new Shooter(hardwareMap);
        transfur = new Transfur(hardwareMap);

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
        follower.setPose(ctx.pose(p.of(9.000, 127.332, -90.000)));
        paths = new Paths(follower, ctx); // Build paths for that alliance
    }

    @Override
    public void loop() {
        follower.update(); // Update follower
        shooter.update(); // Hold flywheel target (must run every loop)
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Driver Station
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", follower.pose().heading());
        telemetry.addData("Flywheel Target", shooter.getTarget());
        telemetry.addData("Flywheel Actual", shooter.getVelocity());
        telemetry.addData("Shooter Ready", shooter.isReady());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop();
        transfur.stop();
    }

    public static class Paths {

        private static final PoseFactory p = PoseFactory.degrees();
        public Path DriveToShoot;
        public Path line2;

        public Paths(Follower follower, AutoContext ctx) {

                    DriveToShoot = curve(
                            p.of(8.000, 136.000, 0.0),
                            p.of(32.343, 116.959, 0.0),
                            p.of(60.499, 118.853, 0.0)
                    ).linear(Math.toRadians(0), Math.toRadians(-90));

                    line2 = line(
                            p.of(60.499, 118.853, 0.0),
                            p.of(12.066, 117.139, 0.0)
                    ).constant(Math.toRadians(-90));
                }
            }


            public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setPathState(1);
                break;
            case 1:
                if (pathTimer.milliseconds() > 0.00) {
                    setPathState(2);
                }
                break;
            case 2:
                shooter.setTarget(SHOOT_SPEED); // Spin up while driving
                follower.follow(paths.DriveToShoot);
                setPathState(3);
                break;
            case 3:
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;
            case 4:
                // Wait for flywheel, with a 1.5s timeout so we never hang
                if (shooter.isReady() || pathTimer.milliseconds() > 1500) {
                    transfur.run();
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.milliseconds() > 1000) { // Feed time, tune this
                    transfur.stop();
                    shooter.setTarget(0);
                    setPathState(6);
                }
                break;
            case 6:
                follower.follow(paths.line2);
                setPathState(7);
                break;
            case 7:
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;
            case 8:
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