package bot.den.frc2026.quaternion.subsystems.shooter;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Orchestra;

import bot.den.frc2026.quaternion.subsystems.CanBeAnInstrument;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase implements CanBeAnInstrument {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private Angle hoodPositionSetpoint = Radians.zero();
    private AngularVelocity shooterVelocitySetpoint = RadiansPerSecond.zero();
    private AngularVelocity feederVelocitySetpoint = RadiansPerSecond.zero();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter/Hood Position", inputs.hoodPositionRot.in(Degrees));
        Logger.recordOutput("Shooter/Hood Setpoint", hoodPositionSetpoint);
        Logger.recordOutput("Shooter/Shooter Velocity", inputs.shooterVelocityRotPerSec.in(RotationsPerSecond));
        Logger.recordOutput("Shooter/Shooter Velocity Setpoint", shooterVelocitySetpoint);
        Logger.recordOutput("Shooter/Feeder Velocity", inputs.feederVelocityRotPerSec.in(RotationsPerSecond));
        Logger.recordOutput("Shooter/Feeder Velocity Setpoint", feederVelocitySetpoint);
    }

    public void addInstruments(Orchestra orchestra) {
        if (io instanceof CanBeAnInstrument instrument) {
            instrument.addInstruments(orchestra);
        }
    }

    public void setHoodAngle(Angle angle) {
        io.setHoodAngleDegrees(angle);
        hoodPositionSetpoint = angle;
    }

    public Angle getHoodSetpoint() {
        return hoodPositionSetpoint;
    }

    public Angle getHoodPosition() {
        return inputs.hoodPositionRot;
    }

    public AngularVelocity getHoodVelocity() {
        return inputs.hoodVelocityRotPerSec;
    }

    public AngularVelocity getShooterVelocity() {
        return inputs.shooterVelocityRotPerSec;
    }

    public void setShooterVelocity(AngularVelocity velocity) {
        io.setShooterVelocity(velocity);
        shooterVelocitySetpoint = velocity;
    }

    public AngularVelocity getFeederVelocity() {
        return inputs.feederVelocityRotPerSec;
    }

    public void setFeederVelocity(AngularVelocity velocity) {
        io.setFeederVelocity(velocity);
        feederVelocitySetpoint = velocity;
    }

    public Command runFeederCommand() {
        return this.runEnd(
                () -> setFeederVelocity(RotationsPerSecond.of(60)),
                () -> setFeederVelocity(RotationsPerSecond.of(0)));
    }

    public Command runShooterOnCommand() {
        return Commands.runOnce(() -> setShooterVelocity(RotationsPerSecond.of(5)));
    }

    public Command runShooterOffCommand() {
        return Commands.runOnce(() -> setShooterVelocity(RotationsPerSecond.of(0)));
    }

    public Command hoodAngleCloseCommand() {
        return this.runOnce(() -> setHoodAngle(Degree.of(5)))
                .andThen(Commands.waitUntil(
                        () -> Math.abs(Units.rotationsToDegrees(inputs.hoodClosedLoopErrorRot)) < 0.1));
    }
}
