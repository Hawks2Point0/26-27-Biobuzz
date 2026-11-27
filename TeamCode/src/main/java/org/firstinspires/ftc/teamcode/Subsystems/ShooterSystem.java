package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Shoot once, then shut everything off. Call update() every loop. */
public class ShooterSystem {

    // Tune these
    private static final double FLYSPEED = 1650;   // ticks/sec
    private static final double HOOD_POS = 0.26;
    private static final long SPINUP_MS = 1500;    // wait for flywheel
    private static final long FEED_MS = 2500;      // transfer + intake run time

    private final DcMotorEx shooter, intake, transfur;
    private final Servo hood;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean shooting = false;

    public ShooterSystem(OpMode opMode) {
        shooter = opMode.hardwareMap.get(DcMotorEx.class, "shooter");
        intake = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        transfur = opMode.hardwareMap.get(DcMotorEx.class, "transfur");
        hood = opMode.hardwareMap.get(Servo.class, "hood");

        shooter.setVelocityPIDFCoefficients(81.8, 0.2, 0, 11.9588);
        intake.setDirection(DcMotor.Direction.REVERSE);
    }

    public void startShoot() {
        hood.setPosition(HOOD_POS);
        shooter.setVelocity(FLYSPEED);
        timer.reset();
        shooting = true;
    }

    public boolean isShooting() {
        return shooting;
    }

    public void update() {
        if (!shooting) return;

        double t = timer.milliseconds();

        if (t > SPINUP_MS + FEED_MS) {
            stop();
        } else if (t > SPINUP_MS) {
            transfur.setVelocity(2600);
            intake.setVelocity(2400);
        }
    }

    public void stop() {
        shooting = false;
        shooter.setVelocity(0);
        transfur.setVelocity(0);
        intake.setVelocity(0);
    }
}