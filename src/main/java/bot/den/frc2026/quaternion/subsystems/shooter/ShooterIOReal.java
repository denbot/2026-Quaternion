package bot.den.frc2026.quaternion.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static bot.den.frc2026.quaternion.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CommutationConfigs;
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
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import bot.den.frc2026.quaternion.subsystems.CanBeAnInstrument;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class ShooterIOReal implements ShooterIO, CanBeAnInstrument {
    private final TalonFX shooter = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID);
    private final TalonFXS shooterFeeder = new TalonFXS(ShooterConstants.SHOOTER_FEEDER_MOTOR_ID);
    private final TalonFXS hopperFeeder = new TalonFXS(ShooterConstants.HOPPER_FEEDER_MOTOR_ID);
    private final TalonFX hood = new TalonFX(ShooterConstants.HOOD_MOTOR_ID);

    private static final VelocityTorqueCurrentFOC shooterSpin = new VelocityTorqueCurrentFOC(0)
            .withAcceleration(ShooterConstants.shooterAcceleration);
    private static final VelocityTorqueCurrentFOC shooterFeederSpin = new VelocityTorqueCurrentFOC(0)
            .withAcceleration(ShooterConstants.shooterFeederAcceleration);
    private static final VelocityTorqueCurrentFOC hopperFeederSpin = new VelocityTorqueCurrentFOC(0)
            .withAcceleration(ShooterConstants.shooterFeederAcceleration);

    private final Debouncer shooterConnectedDebounce = new Debouncer(0.5);
    private final Debouncer shooterFeederConnectedDebounce = new Debouncer(0.5);
    private final Debouncer hopperFeederConnectedDebounce = new Debouncer(0.5);
    private final Debouncer hoodConnectedDebounce = new Debouncer(0.5);

    private final StatusSignal<AngularVelocity> shooterVelocityRotPerSec = shooter.getVelocity();
    private final StatusSignal<Current> shooterCurrentAmps = shooter.getSupplyCurrent();
    private final StatusSignal<Double> shooterClosedLoopErrorRot = shooter.getClosedLoopError();

    private final StatusSignal<AngularVelocity> shooterFeederVelocityRotPerSec = shooterFeeder.getVelocity();
    private final StatusSignal<Current> shooterFeederCurrentAmps = shooterFeeder.getSupplyCurrent();

    private final StatusSignal<AngularVelocity> hopperFeederVelocityRotPerSec = hopperFeeder.getVelocity();
    private final StatusSignal<Current> hopperFeederCurrentAmps = hopperFeeder.getSupplyCurrent();

    private final StatusSignal<Angle> hoodPositionRot = hood.getPosition();
    private final StatusSignal<Double> hoodClosedLoopErrorRot = hood.getClosedLoopError();
    private final StatusSignal<AngularVelocity> hoodVelocityRotPerSec = hood.getVelocity();

    public ShooterIOReal() {
        var hoodConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimitEnable(true)
                                .withStatorCurrentLimit(70))
                .withSoftwareLimitSwitch(
                        new SoftwareLimitSwitchConfigs()
                                .withForwardSoftLimitEnable(false)
                                .withForwardSoftLimitThreshold(ShooterConstants.forwardSoftLimit)
                                .withReverseSoftLimitEnable(false)
                                .withReverseSoftLimitThreshold(ShooterConstants.reverseSoftLimit))
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

        var shooterConfig = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimitEnable(true)
                                .withStatorCurrentLimit(60))
                .withSlot0(new Slot0Configs().withKS(5.4).withKP(3));

        var shooterFeederConfig = new TalonFXSConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimitEnable(true)
                                .withStatorCurrentLimit(60))
                .withCommutation(new CommutationConfigs().withMotorArrangement(MotorArrangementValue.Minion_JST))
                .withSlot0(new Slot0Configs().withKS(5.4).withKP(3));

        var hopperFeederConfig = new TalonFXSConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimitEnable(true)
                                .withStatorCurrentLimit(60))
                .withCommutation(new CommutationConfigs().withMotorArrangement(MotorArrangementValue.Minion_JST))
                .withSlot0(new Slot0Configs().withKS(5.4).withKP(3));

        hood.setNeutralMode(NeutralModeValue.Brake);
        shooter.setNeutralMode(NeutralModeValue.Coast);
        shooterFeeder.setNeutralMode(NeutralModeValue.Coast);
        hopperFeeder.setNeutralMode(NeutralModeValue.Coast);

        tryUntilOk(5, () -> hood.getConfigurator().apply(hoodConfig, 0.25));
        tryUntilOk(5, () -> shooter.getConfigurator().apply(shooterConfig, 0.25));
        tryUntilOk(5, () -> shooterFeeder.getConfigurator().apply(shooterFeederConfig, 0.25));
        tryUntilOk(5, () -> hopperFeeder.getConfigurator().apply(hopperFeederConfig, 0.25));

        BaseStatusSignal.setUpdateFrequencyForAll(
                shooter.getIsProLicensed().getValue() ? 200 : 50,
                shooterVelocityRotPerSec,
                shooterCurrentAmps);

        BaseStatusSignal.setUpdateFrequencyForAll(
                shooterFeeder.getIsProLicensed().getValue() ? 200 : 50,
                shooterFeederVelocityRotPerSec,
                shooterFeederCurrentAmps);

        BaseStatusSignal.setUpdateFrequencyForAll(
                hopperFeeder.getIsProLicensed().getValue() ? 200 : 50,
                hopperFeederVelocityRotPerSec,
                hopperFeederCurrentAmps);

        BaseStatusSignal.setUpdateFrequencyForAll(
                hood.getIsProLicensed().getValue() ? 200 : 50,
                hoodPositionRot,
                hoodClosedLoopErrorRot,
                hoodVelocityRotPerSec);

        ParentDevice.optimizeBusUtilizationForAll(shooter, shooterFeeder, hopperFeeder, hood);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        var shooterStatus = BaseStatusSignal.refreshAll(shooterVelocityRotPerSec, shooterCurrentAmps);
        var shooterFeederStatus = BaseStatusSignal.refreshAll(shooterFeederVelocityRotPerSec, shooterFeederCurrentAmps);
        var hopperFeederStatus = BaseStatusSignal.refreshAll(hopperFeederVelocityRotPerSec, hopperFeederCurrentAmps);
        var hoodStatus = BaseStatusSignal.refreshAll(hoodPositionRot, hoodClosedLoopErrorRot,
                hoodVelocityRotPerSec);

        inputs.shooterConnected = shooterConnectedDebounce.calculate(shooterStatus.isOK());
        inputs.shooterFeederConnected = shooterFeederConnectedDebounce.calculate(shooterFeederStatus.isOK());
        inputs.hopperFeederConnected = hopperFeederConnectedDebounce.calculate(hopperFeederStatus.isOK());
        inputs.hoodConnected = hoodConnectedDebounce.calculate(hoodStatus.isOK());

        inputs.shooterVelocityRotPerSec = shooterVelocityRotPerSec.getValue();
        inputs.shooterCurrentAmps = shooterCurrentAmps.getValue();
        inputs.shooterClosedLoopErrorRot = shooterClosedLoopErrorRot.getValue();

        inputs.shooterFeederVelocityRotPerSec = shooterFeederVelocityRotPerSec.getValue();
        inputs.shooterFeederCurrentAmps = shooterFeederCurrentAmps.getValue();

        inputs.hopperFeederVelocityRotPerSec = hopperFeederVelocityRotPerSec.getValue();
        inputs.hopperFeederCurrentAmps = hopperFeederCurrentAmps.getValue();

        inputs.hoodPositionRot = hoodPositionRot.getValue();
        inputs.hoodClosedLoopErrorRot = hoodClosedLoopErrorRot.getValueAsDouble();
        inputs.hoodVelocityRotPerSec = hoodVelocityRotPerSec.getValue();
    }

    public void addInstruments(Orchestra orchestra) {
        orchestra.addInstrument(shooter);
        orchestra.addInstrument(shooterFeeder);
        orchestra.addInstrument(hopperFeeder);
        orchestra.addInstrument(hood);
    }

    /*
     * Set the intake rotator angle.
     * Make sure units of measurement are consistent.
     */
    public void setHoodAngleDegrees(Angle angle) {
        hood.setControl(new PositionVoltage(angle.in(Rotations)));
    }

    /** Set the shooter wheel velocity. */
    public void setShooterVelocity(AngularVelocity velocity) {
        shooter.setControl(shooterSpin.withVelocity(velocity.in(RotationsPerSecond)));
    }

    /** Set the shooter Feeder wheel velocity. */
    public void setshooterFeederVelocity(AngularVelocity velocity) {
        shooterFeeder.setControl(shooterFeederSpin.withVelocity(velocity.in(RotationsPerSecond)));
    }

    /** Set the hopper Feeder wheel velocity. */
    public void setHopperFeederVelocity(AngularVelocity velocity) {
        hopperFeeder.setControl(hopperFeederSpin.withVelocity(velocity.in(RotationsPerSecond)));
    }

    /** Apply a neutral static brake to the shooter rotator motor. */
    public void setStaticBrake() {
        hood.setControl(new StaticBrake());
    }
}
