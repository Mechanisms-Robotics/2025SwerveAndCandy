package frc.robot.subsystems;

public class StateMachine {
    
}

public enum State {
    /**No AprilTag seen by the robot */
    NoTargetIdentified,
    /**AprilTag seen on the reef */
    DetectedReefTarget,
    /**Going towards AprilTag on the reef */
    DrivingToReefTarget,
    /**Arrived at AprilTag on the reef */
    ArrivedAtReefTarget
}

private State state = State.NoTargetIdentified;

public State getState() {
    return state;
}