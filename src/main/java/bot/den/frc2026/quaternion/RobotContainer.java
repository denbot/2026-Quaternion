// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package bot.den.frc2026.quaternion;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import bot.den.foxflow.RobotState;
import bot.den.frc2026.quaternion.commands.DriveCommands;
import bot.den.frc2026.quaternion.generated.TunerConstants;
import bot.den.frc2026.quaternion.rebuilt.HubState;
import bot.den.frc2026.quaternion.rebuilt.IntakeExtensionState;
import bot.den.frc2026.quaternion.rebuilt.IntakeState;
import bot.den.frc2026.quaternion.rebuilt.MatchState;
import bot.den.frc2026.quaternion.rebuilt.Rebuilt;
import bot.den.frc2026.quaternion.rebuilt.RebuiltStateMachine;
import bot.den.frc2026.quaternion.rebuilt.ShooterHoodState;
import bot.den.frc2026.quaternion.rebuilt.ShooterState;
import bot.den.frc2026.quaternion.subsystems.drive.Drive;
import bot.den.frc2026.quaternion.subsystems.drive.GyroIO;
import bot.den.frc2026.quaternion.subsystems.drive.GyroIOPigeon2;
import bot.den.frc2026.quaternion.subsystems.drive.ModuleIO;
import bot.den.frc2026.quaternion.subsystems.drive.ModuleIOSim;
import bot.den.frc2026.quaternion.subsystems.drive.ModuleIOTalonFX;
import bot.den.frc2026.quaternion.subsystems.intake.Intake;
import bot.den.frc2026.quaternion.subsystems.intake.IntakeIO;
import bot.den.frc2026.quaternion.subsystems.intake.IntakeIOReal;
import bot.den.frc2026.quaternion.subsystems.intake.IntakeIOSim;
import bot.den.frc2026.quaternion.subsystems.shooter.Shooter;
import bot.den.frc2026.quaternion.subsystems.shooter.ShooterIO;
import bot.den.frc2026.quaternion.subsystems.shooter.ShooterIOReal;
import bot.den.frc2026.quaternion.subsystems.shooter.ShooterIOSim;

import java.lang.ModuleLayer.Controller;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Shooter shooter;
    private final Intake intake;
    private final RebuiltStateMachine stateMachine = new RebuiltStateMachine(MatchState.NONE, HubState.INACTIVE,
            ShooterState.OFF, ShooterHoodState.NONE, IntakeState.OFF, IntakeExtensionState.IN);

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
                // a CANcoder
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));

                shooter = new Shooter(new ShooterIOReal());
                intake = new Intake(new IntakeIOReal());

                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));

                shooter = new Shooter(new ShooterIOSim());
                intake = new Intake(new IntakeIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        });

                shooter = new Shooter(new ShooterIO() {
                });
                intake = new Intake(new IntakeIO() {
                });
                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();

        // setup state transitions
        HubState.setup(stateMachine);
        MatchState.setup(stateMachine);
        shooter.setup(stateMachine);
        intake.setup(stateMachine);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX()));

        // Lock to 0° when A button is held
        controller
                .a()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                drive,
                                () -> -controller.getLeftY(),
                                () -> -controller.getLeftX(),
                                () -> Rotation2d.kZero));

        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro to 0° when B button is pressed
        controller
                .b()
                .onTrue(
                        Commands.runOnce(
                                () -> drive.setPose(
                                        new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                                drive)
                                .ignoringDisable(true));


        
        stateMachine.state(ShooterState.AT_SPEED).to(ShooterState.SHOOTING).transitionWhen(controller.rightTrigger());
        stateMachine.state(ShooterState.SHOOTING).to(ShooterState.AT_SPEED).transitionWhen(controller.rightTrigger().negate());

        stateMachine.state(ShooterState.OFF).to(ShooterState.SPINNING_UP).transitionWhen(controller.povLeft());
        stateMachine.state(ShooterState.SPINNING_UP).to(ShooterState.OFF).transitionWhen(controller.povLeft());
        stateMachine.state(ShooterState.AT_SPEED).to(ShooterState.OFF).transitionWhen(controller.povLeft());
        stateMachine.state(ShooterState.SHOOTING).to(ShooterState.OFF).transitionWhen(controller.povLeft());

        stateMachine.state(IntakeState.OFF).to(IntakeState.ON).transitionWhen(controller.leftTrigger());
        stateMachine.state(IntakeState.ON).to(IntakeState.OFF).transitionWhen(controller.leftTrigger().negate());

        stateMachine.state(IntakeExtensionState.IN).to(IntakeExtensionState.OUT).transitionWhen(controller.rightBumper());
        stateMachine.state(IntakeExtensionState.OUT).to(IntakeExtensionState.IN).transitionWhen(controller.rightBumper());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public RebuiltStateMachine getStateMachine() {
        return stateMachine;
    }
}
