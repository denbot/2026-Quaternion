package bot.den.frc2026.quaternion.rebuilt;

import bot.den.foxflow.RobotState;
import bot.den.foxflow.StateMachine;

@StateMachine
public record Rebuilt(
        RobotState robotState,
        MatchState matchState,
        HubState hubState,
        ShooterState shooterState,
        ShooterHoodState shooterHoodState
) {
}
