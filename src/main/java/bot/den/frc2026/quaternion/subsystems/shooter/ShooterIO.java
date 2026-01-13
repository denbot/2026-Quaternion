package bot.den.frc2026.quaternion.subsystems.shooter;


import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean shooterConnected = false;
    public AngularVelocity shooterVelocityRotPerSec = RevolutionsPerSecond.zero();
    public Current shooterCurrentAmps = Amp.zero();
    public double shooterClosedLoopErrorRot = 0.0;

    public boolean hoodConnected = false;
    public Angle hoodPositionRot = Degree.zero();
    public double hoodClosedLoopErrorRot = 0.0;
    public AngularVelocity hoodVelocityRotPerSec = RevolutionsPerSecond.zero();

    public boolean shooterFeederConnected = false;
    public AngularVelocity shooterFeederVelocityRotPerSec = RevolutionsPerSecond.zero();
    public Current shooterFeederCurrentAmps = Amp.zero();

    public boolean hopperFeederConnected = false;
    public AngularVelocity hopperFeederVelocityRotPerSec = RevolutionsPerSecond.zero();
    public Current hopperFeederCurrentAmps = Amp.zero();
  }

  //List of methods that each IO Layer should be accounting for
  /** Update the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /* Set the intake rotator angle.
   * Make sure units of measurement are consistent.
  */
  public default void setHoodAngleDegrees(Angle angle) {}
  
  /** Set the shooter wheel velocity. */
  public default void setShooterVelocity(AngularVelocity velocity) {}

  /** Set the shooterFeeder wheel velocity. */
  public default void setShooterFeederVelocity(AngularVelocity velocity) {}

  /** Set the hopperFeeder wheel velocity. */
  public default void setHopperFeederVelocity(AngularVelocity velocity) {}

  /** Apply a neutral static brake to the shooter rotator motor. */
  public default void setStaticBrake() {}
}
