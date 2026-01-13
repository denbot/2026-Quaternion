package bot.den.frc2026.quaternion.subsystems.shooter;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Orchestra;

import bot.den.frc2026.quaternion.rebuilt.RebuiltStateMachine;
import bot.den.frc2026.quaternion.rebuilt.ShooterHoodState;
import bot.den.frc2026.quaternion.rebuilt.ShooterState;
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

    public Command startFeederCommand() {
        return Commands.runOnce(
                () -> setFeederVelocity(RotationsPerSecond.of(60)));
    }

    public Command stopFeederCommand() {
        return Commands.runOnce(
                () -> setFeederVelocity(RotationsPerSecond.of(0)));
    }

    public Command runShooterOnCommand() {
        return Commands.runOnce(() -> setShooterVelocity(RotationsPerSecond.of(5)));
    }

    public Command runShooterOffCommand() {
        return Commands.runOnce(() -> setShooterVelocity(RotationsPerSecond.of(0)));
    }

    public Command setHoodAngleCommand() {
        return this.runOnce(() -> setHoodAngle(hoodPositionSetpoint))
                .andThen(Commands.waitUntil(
                        () -> Math.abs(Units.rotationsToDegrees(inputs.hoodClosedLoopErrorRot)) < 0.1));
    }

    public void setup(RebuiltStateMachine stateMachine) {
        stateMachine.state(ShooterState.SPINNING_UP).to(ShooterState.AT_SPEED).transitionWhen(
                () -> Math.abs(Units.rotationsToDegrees(inputs.shooterClosedLoopErrorRot)) < 1);

        stateMachine.state(ShooterState.AT_SPEED).to(ShooterState.SPINNING_UP).transitionWhen(
                () -> Math.abs(Units.rotationsToDegrees(inputs.shooterClosedLoopErrorRot)) > 1);

        stateMachine.state(ShooterState.OFF).to(ShooterState.SPINNING_UP).transitionAlways().run(runShooterOnCommand());

        stateMachine.state(ShooterState.SPINNING_UP).to(ShooterState.OFF).transitionAlways()
                .run(runShooterOffCommand());
        stateMachine.state(ShooterState.AT_SPEED).to(ShooterState.OFF).transitionAlways().run(runShooterOffCommand());
        stateMachine.state(ShooterState.SHOOTING).to(ShooterState.OFF).transitionAlways().run(runShooterOffCommand());

        stateMachine.state(ShooterState.AT_SPEED).to(ShooterState.SHOOTING).transitionAlways().run(startFeederCommand());
        stateMachine.state(ShooterState.SHOOTING).to(ShooterState.AT_SPEED).transitionAlways().run(stopFeederCommand());

        stateMachine.state(ShooterHoodState.NONE).to(ShooterHoodState.CLOSE_SHOT).transitionAlways().run(setHoodAngleCommand());
    }
}
