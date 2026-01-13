package bot.den.frc2026.quaternion.subsystems.intake;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
    private static final DCMotor intakeMotor = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor extenderMotor = DCMotor.getKrakenX44Foc(1);

    private DCMotorSim intakeSim;
    private DCMotorSim extenderSim;

    private final PIDController intakeController = new PIDController(0.1, 0, 0);
    private final PIDController extenderController = new PIDController(1.0, 0, 0);

    private double intakeAppliedVolts = 0.0;
    private double extenderAppliedVolts = 0.0;

    public IntakeIOSim() {
        intakeSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.008, 1),
                intakeMotor);

        extenderSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.1, 120),
                extenderMotor);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        intakeAppliedVolts = intakeController.calculate(intakeSim.getAngularVelocity().in(RotationsPerSecond));
        extenderAppliedVolts = extenderController.calculate(extenderSim.getAngularPosition().in(Degrees));

        intakeSim.setInputVoltage(MathUtil.clamp(intakeAppliedVolts, -12.0,
                12.0));
        intakeSim.update(0.02);

        extenderSim.setInputVoltage(MathUtil.clamp(extenderAppliedVolts, -12.0,
                12.0));
        extenderSim.update(0.02);

        inputs.intakeConnected = true;
        inputs.extenderConnected = true;

        // Sim Velocity defaults to Rad/sec. 60 rps = 377 Rad/sec
        inputs.intakeVelocityRotPerSec = RotationsPerSecond.of(intakeSim.getAngularVelocityRPM() / 60.0);
        inputs.intakeCurrentAmps = Amp.of(intakeSim.getCurrentDrawAmps());

        inputs.extenderPositionRot = extenderSim.getAngularPosition();
        inputs.extenderVelocityRotPerSec = RotationsPerSecond.of(extenderSim.getAngularVelocityRPM() / 60.0);
        inputs.extenderClosedLoopErrorRot = extenderController.getError();
    }

    /*
     * Set the intake distance angle.
     * Make sure units of measurement are consistent.
     */
    public void setExtenderPosition(Angle angle) {
        extenderController.setSetpoint(angle.in(Degrees));
    }

    /** Set the shooter wheel velocity. */
    public void setIntakeVelocity(AngularVelocity velocity) {
        intakeController.setSetpoint(velocity.in(RotationsPerSecond));
    }

    /** Apply a neutral static brake to the intake extender motor. */
    public void setStaticBrake() {
    }

}