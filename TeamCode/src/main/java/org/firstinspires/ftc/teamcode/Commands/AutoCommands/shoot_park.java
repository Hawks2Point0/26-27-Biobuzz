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

@Autonomous(name = "shoot_park", group = "Autonomous")
public class shoot_park extends OpMode {

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
                new AutoRegistry<AutoContext>().add("shoot_park", c -> c),
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
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;
            case 2:
                setPathState(3);
                break;
            case 3:
                if (pathTimer.milliseconds() > 1000) {
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
