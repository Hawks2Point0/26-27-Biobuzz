package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/** Intake motor. Use run(), reverse(), stop(). */
public class IntakeSubsystem {

    private static final double INTAKE_VEL = 2400;
    private static final double REVERSE_VEL = -2450;

    private final DcMotorEx intake;

    public IntakeSubsystem(OpMode opMode) {
        intake = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void run() {
        intake.setVelocity(INTAKE_VEL);
    }

    public void reverse() {
        intake.setVelocity(REVERSE_VEL);
    }

    public void stop() {
        intake.setVelocity(0);
    }
}