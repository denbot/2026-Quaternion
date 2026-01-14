package bot.den.frc2026.quaternion.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Orchestra;

import bot.den.frc2026.quaternion.rebuilt.IntakeExtensionState;
import bot.den.frc2026.quaternion.rebuilt.IntakeState;
import bot.den.frc2026.quaternion.rebuilt.RebuiltStateMachine;
import bot.den.frc2026.quaternion.subsystems.CanBeAnInstrument;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase implements CanBeAnInstrument {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private Angle extenderPositionSetpoint = Radians.zero();
    private AngularVelocity intakeVelocitySetpoint = RadiansPerSecond.zero();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        Logger.recordOutput("Intake/Extender Position", inputs.extenderPositionRot);
        Logger.recordOutput("Intake/Extender Setpoint", extenderPositionSetpoint);
        Logger.recordOutput("Intake/Intake Velocity", inputs.intakeVelocityRotPerSec.in(RotationsPerSecond));
        Logger.recordOutput("Intake/Intake Velocity Setpoint", intakeVelocitySetpoint);
    }

    public void addInstruments(Orchestra orchestra) {
        if (io instanceof CanBeAnInstrument instrument) {
            instrument.addInstruments(orchestra);
        }
    }

    public void setExtenderAngle(Angle angle) {
        io.setExtenderPosition(angle);
        extenderPositionSetpoint = angle;
    }

    public Angle getExtenderSetpoint() {
        return extenderPositionSetpoint;
    }

    public Angle getExtenderPosition() {
        return inputs.extenderPositionRot;
    }

    public AngularVelocity getExtenderVelocity() {
        return inputs.extenderVelocityRotPerSec;
    }

    public AngularVelocity getIntakeVelocity() {
        return inputs.intakeVelocityRotPerSec;
    }

    public void setIntakeVelocity(AngularVelocity velocity) {
        io.setIntakeVelocity(velocity);
        intakeVelocitySetpoint = velocity;
    }

    public Command runIntakeOnCommand() {
        return Commands.runOnce(() -> setIntakeVelocity(RotationsPerSecond.of(IntakeConstants.intakeSpeed)));
    }

    public Command runIntakeOffCommand() {
        return Commands.runOnce(() -> setIntakeVelocity(RotationsPerSecond.of(0)));
    }

    public Command setExtenderOutCommand() {
        return this.runOnce(() -> setExtenderAngle(Rotations.of(6.7*9)))
                .andThen(Commands.waitUntil(
                        () -> Math.abs(Units.rotationsToDegrees(inputs.extenderClosedLoopErrorRot)) < 0.1));
    }

    public Command setExtenderInCommand() {
        return this.runOnce(() -> setExtenderAngle(Rotations.of(0)))
                .andThen(Commands.waitUntil(
                        () -> Math.abs(Units.rotationsToDegrees(inputs.extenderClosedLoopErrorRot)) < 0.1));
    }

    public void setup(RebuiltStateMachine stateMachine) {
        stateMachine.state(IntakeState.OFF).to(IntakeState.ON).run(runIntakeOnCommand());
        stateMachine.state(IntakeState.ON).to(IntakeState.OFF).run(runIntakeOffCommand());
        stateMachine.state(IntakeExtensionState.IN).to(IntakeExtensionState.OUT).run(setExtenderOutCommand());
        stateMachine.state(IntakeExtensionState.OUT).to(IntakeExtensionState.IN).run(setExtenderInCommand());
    }
}
