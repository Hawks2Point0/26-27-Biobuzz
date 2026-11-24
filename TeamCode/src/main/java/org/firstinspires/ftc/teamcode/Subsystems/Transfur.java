package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/** Controls the transfer and intake motors. */
public class Transfur {

    private static final double FEED_VELOCITY = 2600;
    private static final double INTAKE_VELOCITY = 2400;
    private static final double REVERSE_INTAKE_VELOCITY = -2450;
    private static final double INTAKE_CURRENT_LIMIT_AMPS = 3.45;

    private final DcMotorEx transferMotor;
    private final DcMotorEx intakeMotor;
    private boolean intakeLockedOut;

    public Transfur(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfur");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setCurrentAlert(INTAKE_CURRENT_LIMIT_AMPS, CurrentUnit.AMPS);
    }

    public void run() {
        transferMotor.setVelocity(FEED_VELOCITY);
    }

    /** Run the transfer at a caller-selected velocity, in encoder ticks per second. */
    public void run(double velocity) {
        transferMotor.setVelocity(velocity);
    }

    public void stopTransfer() {
        transferMotor.setVelocity(0);
    }

    /** Run intake forward; call repeatedly in an autonomous loop to check its current lockout. */
    public void runIntake() {
        updateIntake(false, false);
    }

    /** Run intake backward to clear a jam. */
    public void reverseIntake() {
        updateIntake(true, false);
    }

    public void stopIntake() {
        intakeMotor.setVelocity(0);
    }

    /** Run intake and transfer together. Call from a loop to keep checking intake current. */
    public void runBoth() {
        run();
        runIntake();
    }

    /** Clear the intake's overcurrent lockout before another autonomous intake step. */
    public void resetIntakeLockout() {
        intakeLockedOut = false;
    }

    /** Update intake each loop. Reverse overrides the current lockout. */
    public void updateIntake(boolean reverse, boolean resetLockout) {
        if (resetLockout) {
            intakeLockedOut = false;
        }

        if (reverse) {
            intakeMotor.setVelocity(REVERSE_INTAKE_VELOCITY);
            return;
        }

        if (!intakeLockedOut && intakeMotor.isOverCurrent()) {
            intakeLockedOut = true;
        }

        intakeMotor.setVelocity(intakeLockedOut ? 0 : INTAKE_VELOCITY);
    }

    public boolean isIntakeLockedOut() {
        return intakeLockedOut;
    }

    public double getTransferVelocity() {
        return transferMotor.getVelocity();
    }

    public double getIntakeVelocity() {
        return intakeMotor.getVelocity();
    }

    /** Stop both motors. */
    public void stop() {
        transferMotor.setVelocity(0);
        intakeMotor.setVelocity(0);
    }
}