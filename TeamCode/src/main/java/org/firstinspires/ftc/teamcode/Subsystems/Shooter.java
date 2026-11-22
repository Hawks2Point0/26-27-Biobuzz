package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Shooter {

    public static final double MIN_SPEED = 0;
    public static final double MAX_SPEED = 2600;

    // Hybrid control
    private static final double BB_FULL_POWER = 1.0;   // power when far below target
    private static final double BB_HOLD_POWER = 0.0;   // power when far above target
    private static final double VEL_ENTER_BAND = 60;   // switch TO velocity control within this many ticks/sec
    private static final double VEL_EXIT_BAND = 120;   // switch BACK to bang-bang past this
    private static final double READY_TOLERANCE = 100; // "ready" = this close below target

    // Flywheel velocity PIDF
    private static final double P = 1.17;
    private static final double I = 0.117;
    private static final double D = 0;
    private static final double F = 11.7;

    private final DcMotorEx motor;
    private double target = 0;
    private boolean inVelocityControl = false;
    private boolean encoderMode = true;

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "shooter");
        motor.setVelocityPIDFCoefficients(P, I, D, F);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Set the flywheel target in ticks/sec (clamped). 0 = off. */
    public void setTarget(double ticksPerSec) {
        target = Math.max(MIN_SPEED, Math.min(MAX_SPEED, ticksPerSec));
    }

    public double getTarget() {
        return target;
    }

    /** Call every loop. Runs the bang-bang / velocity-hold logic. */
    public void update() {
        double velocity = motor.getVelocity();
        double error = target - velocity;

        if (target <= 0) {
            inVelocityControl = false;
            setEncoderMode(false);
            motor.setPower(0);
            return;
        }

        // Hysteresis between the two control modes
        if (!inVelocityControl && Math.abs(error) <= VEL_ENTER_BAND) {
            inVelocityControl = true;
        } else if (inVelocityControl && Math.abs(error) > VEL_EXIT_BAND) {
            inVelocityControl = false;
        }

        if (inVelocityControl) {
            setEncoderMode(true);
            motor.setVelocity(target);
        } else {
            setEncoderMode(false);
            motor.setPower(error > 0 ? BB_FULL_POWER : BB_HOLD_POWER);
        }
    }

    public boolean isReady() {
        return motor.getVelocity() >= target - READY_TOLERANCE;
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public double getPower() {
        return motor.getPower();
    }

    public double getCurrentAmps() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    public boolean isVelocityControl() {
        return inVelocityControl;
    }

    public void stop() {
        target = 0;
        inVelocityControl = false;
        setEncoderMode(true);
        motor.setVelocity(0);
    }

    // Only touches the motor when the mode actually needs to change
    private void setEncoderMode(boolean useEncoder) {
        if (useEncoder == encoderMode) return;
        motor.setMode(useEncoder
                ? DcMotor.RunMode.RUN_USING_ENCODER
                : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderMode = useEncoder;
    }
}