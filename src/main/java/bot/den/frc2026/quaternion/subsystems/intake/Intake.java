package bot.den.frc2026.quaternion.subsystems.intake;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
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
import edu.wpi.first.units.measure.Distance;
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

    public AngularVelocity getIntakerVelocity() {
        return inputs.intakeVelocityRotPerSec;
    }

    public void setIntakeVelocity(AngularVelocity velocity) {
        io.setIntakeVelocity(velocity);
        intakeVelocitySetpoint = velocity;
    }

    public Command runIntakeOnCommand() {
        return Commands.runOnce(() -> setIntakeVelocity(RotationsPerSecond.of(5)));
    }

    public Command runIntakeOffCommand() {
        return Commands.runOnce(() -> setIntakeVelocity(RotationsPerSecond.of(0)));
    }

    public Command setHoodAngleCommand() {
        return this.runOnce(() -> setExtenderAngle(extenderPositionSetpoint))
                .andThen(Commands.waitUntil(
                        () -> Math.abs(Units.rotationsToDegrees(inputs.extenderClosedLoopErrorRot)) < 0.1));
    }

    public void setup(RebuiltStateMachine stateMachine) {
        
    }
}
