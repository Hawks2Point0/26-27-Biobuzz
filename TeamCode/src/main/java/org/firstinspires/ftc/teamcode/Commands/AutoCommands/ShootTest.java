package org.firstinspires.ftc.teamcode.Commands.AutoCommands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSystem;

/** Press Start and it shoots once automatically, then stops. */
@Autonomous(name = "Shoot Test", group = "Test")
public class ShootTest extends OpMode {

    private ShooterSystem shooter;

    @Override
    public void init() {
        shooter = new ShooterSystem(this);
    }

    @Override
    public void start() {
        shooter.startShoot();
    }

    @Override
    public void loop() {
        shooter.update();

        telemetry.addData("Shooting", shooter.isShooting());
        telemetry.update();

        if (!shooter.isShooting()) requestOpModeStop();
    }

    @Override
    public void stop() {
        shooter.stop();
    }
}