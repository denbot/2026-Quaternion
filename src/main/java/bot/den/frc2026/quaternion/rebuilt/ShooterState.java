package bot.den.frc2026.quaternion.rebuilt;

import java.util.Set;

import bot.den.foxflow.LimitsStateTransitions;

public enum ShooterState implements LimitsStateTransitions<ShooterState>{
    OFF,
    SPINNING_UP,
    AT_SPEED,
    SHOOTING;

    public static void setup(RebuiltStateMachine stateMachine) {
        stateMachine.state(MatchState.TRANSITION_SHIFT, OFF).to(SPINNING_UP).transitionAlways();
    }

    @Override
    public boolean canTransitionState(ShooterState newState) {
        return (switch (this) {
            case OFF -> Set.of(SPINNING_UP);
            case SPINNING_UP -> Set.of(OFF, AT_SPEED);
            case AT_SPEED -> Set.of(OFF, SPINNING_UP, SHOOTING);
            case SHOOTING -> Set.of(OFF, AT_SPEED);
        }).contains(newState);
    }
}
