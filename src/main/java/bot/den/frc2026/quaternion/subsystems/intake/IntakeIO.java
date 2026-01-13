package bot.den.frc2026.quaternion.subsystems.intake;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean intakeConnected = false;
        public AngularVelocity intakeVelocityRotPerSec = RevolutionsPerSecond.zero();
        public Current intakeCurrentAmps = Amp.zero();

        public boolean extenderConnected = false;
        public Angle extenderPositionRot = Degree.zero();
        public double extenderClosedLoopErrorRot = 0.0;
        public AngularVelocity extenderVelocityRotPerSec = RevolutionsPerSecond.zero();
    }

    // List of methods that each IO Layer should be accounting for
    /** Update the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {
    }

    /*
     * Set the intake extender angle.
     * Make sure units of measurement are consistent.
     */
    public default void setExtenderPosition(Angle angle) {
    }

    /** Set the shooter wheel velocity. */
    public default void setIntakeVelocity(AngularVelocity velocity) {
    }

    /** Apply a neutral static brake to the extender motor. */
    public default void setStaticBrake() {
    }
}
