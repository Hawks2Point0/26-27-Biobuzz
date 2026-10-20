
package org.firstinspires.ftc.teamcode.pedro;

import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;
import com.pedropathing.tuning.autotune.Tuner;
import com.pedropathing.tuning.autotune.Procedure;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.revhub.drivetrains.Mecanum;

public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }
    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }
    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner((hardwareMap) -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig), (hardwareMap) -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    }
    @Tuner
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(
                        hardwareMap, Constants.drivetrainConfig
                ),
                hardwareMap -> new PinpointLocalizer(
                        hardwareMap, Constants.localizerConfig
                ),
                null
        );
    }

}
