private void shooter2() {

    // Toggle Auto / Manual with A
    boolean a = gamepad1.a;

    if (a && !previousA) {
        autoFlywheel = !autoFlywheel;
    }

    previousA = a;

    boolean up = gamepad1.dpad_up;
    boolean down = gamepad1.dpad_down;
    boolean y = gamepad1.y;

    if (autoFlywheel) {

        // =========================
        // AUTOMATIC DISTANCE CONTROL
        // LIMELIGHT DISABLED
        // =========================

        /*
        if (limelightHasTarget && limelightDistance > 0) {
            flyspeed = calculateAutoFlywheelSpeed(
                    limelightDistance
            );
        }
        */

    } else {

        // Manual speed adjustment
        if (up && !previousUp) {
            flyspeed += SPEED_STEP;
        }

        if (down && !previousDown) {
            flyspeed -= SPEED_STEP;
        }

        if (y && !previousY) {
            flyspeed = 1550;
        }
    }

    previousUp = up;
    previousDown = down;
    previousY = y;

    flyspeed = Math.max(
            MIN_FLYSPEED,
            Math.min(MAX_FLYSPEED, flyspeed)
    );

    shooter.setVelocity(flyspeed);
}
