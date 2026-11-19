
package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    // ==============================
    // DRIVETRAIN CONFIGURATION
    // ==============================

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {

        c.frontLeftName.set("fl");
        c.frontRightName.set("fr");
        c.backLeftName.set("bl");
        c.backRightName.set("br");

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

    });

    // ==============================
    // PINPOINT LOCALIZER CONFIGURATION
    // ==============================

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {

        c.name.set("pinpoint");

        c.podType.set(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        c.xPodOffset.set(-0.00890530176519409);
        c.yPodOffset.set(-0.013852692025852955);

        c.xPodDirection.set(
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        c.yPodDirection.set(
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);

    });

    // ==============================
    // FORESIGHT CONFIGURATION
    // ==============================

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {

        Controller primaryTranslationalForward =
                Controller.proportional(0.22964694365612215);

        Controller secondaryTranslationalForward =
                Controller.proportional(0.08484840940158916);

        Controller primaryTranslationalLateral =
                Controller.proportional(0.3345945836796959);

        Controller secondaryTranslationalLateral =
                Controller.proportional(0.12362375813771159);

        // Forward and strafe controllers

        c.forwardTranslational.set(
                Controller.piecewise(secondaryTranslationalForward)
                        .put(2.5, primaryTranslationalForward)
        );

        c.strafeTranslational.set(
                Controller.piecewise(secondaryTranslationalLateral)
                        .put(2.5, primaryTranslationalLateral)
        );

        // Coast and braking

        c.coast.set(
                Controller.proportionalFeedforward(0.015033146884223308)
        );

        c.brake.set(
                Controller.proportionalFeedforward(0.012778174851589812)
        );

        // Heading control

        c.headingFeedback.set(
                Controller.proportional(3.435947659189172)
        );

        c.headingBrakeCoefficients.set(
                Vector2D.cartesian(
                        0.04997167320032773,
                        0.004248504216009385
                )
        );

        // Linear and quadratic braking

        c.linearBrakeCoefficients.set(
                Matrix.diag(
                        0.09658262873797059,
                        0.06393596093612576
                )
        );

        c.quadraticBrakeCoefficients.set(
                Matrix.diag(
                        0.0010592410116518466,
                        0.0016608203280175492
                )
        );

        // Maximum velocities

        c.maxAchievableForwardVelocity.set(61.18421888199106);
        c.maxAchievableStrafeVelocity.set(51.165589088754906);

        // Natural deceleration

        c.naturalForwardDeceleration.set(44.102016031671305);
        c.naturalStrafeDeceleration.set(71.71187736452431);

    });

    // ==============================
    // FOLLOWER CREATION
    // ==============================

    public static Follower create(HardwareMap h) {

        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );

    }
}
