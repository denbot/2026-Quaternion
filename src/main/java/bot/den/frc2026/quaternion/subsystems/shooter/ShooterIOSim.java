package bot.den.frc2026.quaternion.subsystems.shooter;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO{
    private static final DCMotor shooterMotor = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor shooterFeederMotor = DCMotor.getMinion(1);
    private static final DCMotor hoodMotor = DCMotor.getKrakenX44Foc(1);

    private DCMotorSim shooterSim;
    private DCMotorSim shooterFeederSim;
    private DCMotorSim hoodSim;

    private final PIDController shooterController = new PIDController(0.1, 0, 0);
    private final PIDController shooterFeederController = new PIDController(1.0, 0, 0);
    private final PIDController hoodController = new PIDController(1.0, 0, 0);

    private double shooterAppliedVolts = 0.0;
    private double shooterFeederAppliedVolts = 0.0;
    private double hoodAppliedVolts = 0.0;

    public ShooterIOSim() {
        shooterSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.008, 1),
                shooterMotor);

        shooterFeederSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.008, 1),
                shooterFeederMotor);

        hoodSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.1, 120),
                hoodMotor);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        shooterAppliedVolts = shooterController.calculate(shooterSim.getAngularVelocity().in(RotationsPerSecond));
        shooterFeederAppliedVolts = shooterFeederController.calculate(shooterFeederSim.getAngularPosition().in(Degrees));
        hoodAppliedVolts = hoodController.calculate(hoodSim.getAngularPosition().in(Degrees));

        shooterSim.setInputVoltage(MathUtil.clamp(shooterAppliedVolts, -12.0,
                12.0));
        shooterSim.update(0.02);

        shooterFeederSim.setInputVoltage(MathUtil.clamp(shooterFeederAppliedVolts, -12.0,
                12.0));
        shooterFeederSim.update(0.02);

        hoodSim.setInputVoltage(MathUtil.clamp(hoodAppliedVolts, -12.0,
                12.0));
        hoodSim.update(0.02);

        inputs.shooterConnected = true;
        inputs.shooterFeederConnected = true;
        inputs.hoodConnected = true;

        // Sim Velocity defaults to Rad/sec. 60 rps = 377 Rad/sec
        inputs.shooterVelocityRotPerSec = RotationsPerSecond.of(shooterSim.getAngularVelocityRPM() / 60.0);
        inputs.shooterCurrentAmps = Amp.of(shooterSim.getCurrentDrawAmps());
        inputs.shooterClosedLoopErrorRot = shooterController.getError();

        inputs.shooterFeederVelocityRotPerSec = RotationsPerSecond.of(shooterFeederSim.getAngularVelocityRPM() / 60.0);
        inputs.shooterFeederCurrentAmps = Amp.of(shooterFeederSim.getCurrentDrawAmps());

        inputs.hoodPositionRot = hoodSim.getAngularPosition();
        inputs.hoodVelocityRotPerSec = RotationsPerSecond.of(hoodSim.getAngularVelocityRPM() / 60.0);
        inputs.hoodClosedLoopErrorRot = hoodController.getError();
    }

    /* Set the hood rotation angle.
    * Make sure units of measurement are consistent.
    */
    public void setHoodAngleDegrees(Angle angle) {
        hoodController.setSetpoint(angle.in(Degrees));
    }
    
    /** Set the shooter wheel velocity. */
    public void setShooterVelocity(AngularVelocity velocity) {
        shooterController.setSetpoint(velocity.in(RotationsPerSecond));
    }

    /** Set the shooterFeeder wheel velocity. */
    public void setshooterFeederVelocity(AngularVelocity velocity) {
        shooterFeederController.setSetpoint(velocity.in(RotationsPerSecond));
    }

    /** Apply a neutral static brake to the shooter rotator motor. */
    public void setStaticBrake() {}

}