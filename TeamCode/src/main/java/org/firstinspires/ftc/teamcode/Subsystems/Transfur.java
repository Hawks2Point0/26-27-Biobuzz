package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfur {

    private static final double FEED_VELOCITY = 2600;

    private final DcMotorEx motor;

    public Transfur(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "transfur");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void run() {
        motor.setVelocity(FEED_VELOCITY);
    }

    public void stop() {
        motor.setVelocity(0);
    }

    public double getVelocity() {
        return motor.getVelocity();
    }
}