package bot.den.frc2026.quaternion.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static bot.den.frc2026.quaternion.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import bot.den.frc2026.quaternion.subsystems.CanBeAnInstrument;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class IntakeIOReal implements IntakeIO, CanBeAnInstrument {
    private final TalonFX intake = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
    private final TalonFX extender = new TalonFX(IntakeConstants.EXTENDER_MOTOR_ID);

    private static final VelocityTorqueCurrentFOC intakeSpin = new VelocityTorqueCurrentFOC(0)
            .withAcceleration(IntakeConstants.intakeAcceleration);

    private final Debouncer intakeConnectedDebounce = new Debouncer(0.5);
    private final Debouncer extenderConnectedDebounce = new Debouncer(0.5);

    private final StatusSignal<AngularVelocity> intakeVelocityRotPerSec = intake.getVelocity();
    private final StatusSignal<Current> intakeCurrentAmps = intake.getSupplyCurrent();

    private final StatusSignal<Angle> extenderPositionRot = extender.getPosition();
    private final StatusSignal<Double> extenderClosedLoopErrorRot = extender.getClosedLoopError();
    private final StatusSignal<AngularVelocity> extenderVelocityRotPerSec = extender.getVelocity();

    public IntakeIOReal() {
        var extenderConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimitEnable(true)
                                .withStatorCurrentLimit(70))
                .withSoftwareLimitSwitch(
                        new SoftwareLimitSwitchConfigs()
                                .withForwardSoftLimitEnable(false)
                                .withForwardSoftLimitThreshold(IntakeConstants.forwardSoftLimit)
                                .withReverseSoftLimitEnable(false)
                                .withReverseSoftLimitThreshold(IntakeConstants.reverseSoftLimit))
                .withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicAcceleration(4)
                                .withMotionMagicCruiseVelocity(2))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(45)
                                .withKD(0)
                                .withKG(0.2)
                                .withGravityType(GravityTypeValue.Elevator_Static));

        var intakeConfig = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimitEnable(true)
                                .withStatorCurrentLimit(60))
                .withSlot0(new Slot0Configs().withKS(5.4).withKP(3));

        extender.setNeutralMode(NeutralModeValue.Brake);
        intake.setNeutralMode(NeutralModeValue.Coast);

        tryUntilOk(5, () -> extender.getConfigurator().apply(extenderConfig, 0.25));
        tryUntilOk(5, () -> intake.getConfigurator().apply(intakeConfig, 0.25));

        BaseStatusSignal.setUpdateFrequencyForAll(
                intake.getIsProLicensed().getValue() ? 200 : 50,
                intakeVelocityRotPerSec,
                intakeCurrentAmps);

        BaseStatusSignal.setUpdateFrequencyForAll(
                extender.getIsProLicensed().getValue() ? 200 : 50,
                extenderPositionRot,
                extenderClosedLoopErrorRot,
                extenderVelocityRotPerSec);

        ParentDevice.optimizeBusUtilizationForAll(intake, extender);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        var intakeStatus = BaseStatusSignal.refreshAll(intakeVelocityRotPerSec, intakeCurrentAmps);
        var extenderStatus = BaseStatusSignal.refreshAll(extenderPositionRot, extenderClosedLoopErrorRot,
                extenderVelocityRotPerSec);

        inputs.intakeConnected = intakeConnectedDebounce.calculate(intakeStatus.isOK());
        inputs.extenderConnected = extenderConnectedDebounce.calculate(extenderStatus.isOK());

        inputs.intakeVelocityRotPerSec = intakeVelocityRotPerSec.getValue();
        inputs.intakeCurrentAmps = intakeCurrentAmps.getValue();

        inputs.extenderPositionRot = extenderPositionRot.getValue();
        inputs.extenderClosedLoopErrorRot = extenderClosedLoopErrorRot.getValueAsDouble();
        inputs.extenderVelocityRotPerSec = extenderVelocityRotPerSec.getValue();
    }

    public void addInstruments(Orchestra orchestra) {
        orchestra.addInstrument(intake);
        orchestra.addInstrument(extender);
    }

    /*
     * Set the intake rotator angle.
     * Make sure units of measurement are consistent.
     */
    public void setextenderAngleDegrees(Angle angle) {
        extender.setControl(new PositionVoltage(angle.in(Rotations)));
    }

    /** Set the intake wheel velocity. */
    public void setintakeVelocity(AngularVelocity velocity) {
        intake.setControl(intakeSpin.withVelocity(velocity.in(RotationsPerSecond)));
    }

    /** Apply a neutral static brake to the intake rotator motor. */
    public void setStaticBrake() {
        extender.setControl(new StaticBrake());
    }
}
