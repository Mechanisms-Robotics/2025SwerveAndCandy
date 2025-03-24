package frc.robot.subsystems;

public class StateMachine {

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

    public StateMachine(State myState) {
        this.myState = myState;
        myState = State.NoTargetIdentified;
    }

    private State myState = State.NoTargetIdentified;

    public State getState() {
        return myState;
    }

    public void setState(State newState) {
        myState = newState;
        return;
    }
}