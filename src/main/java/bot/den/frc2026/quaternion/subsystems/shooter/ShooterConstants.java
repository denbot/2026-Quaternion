package bot.den.frc2026.quaternion.subsystems.shooter;

public class ShooterConstants {
    // TODO: definitley wrong
    public static final double forwardSoftLimit = 0.25;
    public static final double reverseSoftLimit = 0;

    public static final int FEEDER_MOTOR_ID = 31;
    public static final int SHOOTER_MOTOR_ID = 32;
    public static final int HOOD_MOTOR_ID = 33;

    public static final double hoodCloseAngle = 0;

    public static final double shooterSpeed = 60; // Rotations / Second
    public static final double shooterAcceleration = 13; // Rotations / Second^2

    public static final double feederSpeed = 60; // Rotations / Second
    public static final double feederAcceleration = 13; // Rotations / Second^2
}
