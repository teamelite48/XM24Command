package frc.robot.subsystems.arm;

import frc.robot.Constants;

public enum ArmPosition {
    Home(Constants.ARM_HOME_DEGREES),
    Score(Constants.ARM_SCORE_DEGREES),
    Floor(Constants.ARM_FLOOR_DEGREES);

    public final double degrees;

    private ArmPosition(double degrees) {
        this.degrees = degrees;
    }
}
