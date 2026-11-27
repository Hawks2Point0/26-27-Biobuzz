/* ============================================================= *
 *                 Turtle Tracer — Auto-Generated                *
 *                                                               *
 *  Version: 2.7.1.                                              *
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
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "2tip_park", group = "Autonomous")
public class Auto2tip_park extends OpMode {

    public Follower follower; // Pathing follower instance
    private final PoseFactory p = PoseFactory.degrees();
    private int pathState; // Current autonomous path state (state machine)
    private ElapsedTime pathTimer; // Timer for path state machine
    private Paths paths; // Paths defined in the Paths class
    private AutoSelector<AutoContext> selector; // Alliance menu shown during init
    private AutoContext ctx; // The alliance picked, to mirror the paths for

    @Override
    public void init() {
        // ...
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        follower = Constants.create(hardwareMap);

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
        follower.setPose(ctx.pose(p.of(9.000, 25.000, 0.000)));
        paths = new Paths(follower, ctx); // Build paths for that alliance
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
        telemetry.update();
    }

    public static class Paths {

        private static final PoseFactory p = PoseFactory.degrees();
        public Path DriveToShoot;
        public Path line2;
        public Path line3;
        public Path line4;
        public Path line5;
        public Path line6;
        public Path line7;
        public Path line8;

        public Paths(Follower follower, AutoContext ctx) {
            DriveToShoot = curve(
                    ctx.pose(p.of(9.000, 25.000, 0.0)),
                    ctx.pose(p.of(34.272, 24.990, 0.0)),
                    ctx.pose(p.of(60.000, 25.000, 0.0))
            ).linear(ctx.heading(Math.toRadians(0)), ctx.heading(Math.toRadians(90)));

            line2 = line(
                    ctx.pose(p.of(60.000, 25.000, 0.0)),
                    ctx.pose(p.of(60.000, 10.000, 0.0))
            ).linear(
                    ctx.heading(Math.toRadians(90)),
                    ctx.heading(Math.toRadians(180))
            );

            line3 = curve(
                    ctx.pose(p.of(60.000, 10.000, 0.0)),
                    ctx.pose(p.of(36.303, 10.198, 0.0)),
                    ctx.pose(p.of(10.000, 10.000, 0.0))
            ).constant(ctx.heading(Math.toRadians(180)));

            line4 = curve(
                    ctx.pose(p.of(10.000, 10.000, 0.0)),
                    ctx.pose(p.of(22.779, 66.562, 0.0)),
                    ctx.pose(p.of(17.884, 92.077, 0.0)),
                    ctx.pose(p.of(60.000, 120.884, 0.0))
            ).linear(
                    ctx.heading(Math.toRadians(180)),
                    ctx.heading(Math.toRadians(-90))
            );

            line5 = line(
                    ctx.pose(p.of(60.000, 120.884, 0.0)),
                    ctx.pose(p.of(49.000, 120.884, 0.0))
            ).linear(
                    ctx.heading(Math.toRadians(90)),
                    ctx.heading(Math.toRadians(90))
            );

            line6 = line(
                    ctx.pose(p.of(49.000, 120.884, 0.0)),
                    ctx.pose(p.of(49.000, 128.770, 0.0))
            ).linear(
                    ctx.heading(Math.toRadians(90)),
                    ctx.heading(Math.toRadians(90))
            );

            line7 = curve(
                    ctx.pose(p.of(49.000, 128.770, 0.0)),
                    ctx.pose(p.of(49.000, 119.456, 0.0)),
                    ctx.pose(p.of(60.000, 123.592, 0.0))
            ).linear(
                    ctx.heading(Math.toRadians(90)),
                    ctx.heading(Math.toRadians(-90))
            );

            line8 = line(
                    ctx.pose(p.of(60.000, 123.592, 0.0)),
                    ctx.pose(p.of(12.100, 115.726, 0.0))
            ).tangent();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.follow(paths.DriveToShoot);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;
            case 2:
                follower.follow(paths.line2);
                setPathState(3);
                break;
            case 3:
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;
            case 4:
                follower.follow(paths.line3);
                setPathState(5);
                break;
            case 5:
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;
            case 6:
                setPathState(7);
                break;
            case 7:
                if (pathTimer.milliseconds() > 250) {
                    setPathState(8);
                }
                break;
            case 8:
                follower.follow(paths.line4);
                setPathState(9);
                break;
            case 9:
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;
            case 10:
                setPathState(11);
                break;
            case 11:
                if (pathTimer.milliseconds() > 500) {
                    setPathState(12);
                }
                break;
            case 12:
                follower.follow(paths.line5);
                setPathState(13);
                break;
            case 13:
                if (!follower.isBusy()) {
                    setPathState(14);
                }
                break;
            case 14:
                follower.follow(paths.line6);
                setPathState(15);
                break;
            case 15:
                if (!follower.isBusy()) {
                    setPathState(16);
                }
                break;
            case 16:
                follower.follow(paths.line7);
                setPathState(17);
                break;
            case 17:
                if (!follower.isBusy()) {
                    setPathState(18);
                }
                break;
            case 18:
                setPathState(19);
                break;
            case 19:
                if (pathTimer.milliseconds() > 500) {
                    setPathState(20);
                }
                break;
            case 20:
                follower.follow(paths.line8);
                setPathState(21);
                break;
            case 21:
                if (!follower.isBusy()) {
                    setPathState(22);
                }
                break;
            case 22:
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
