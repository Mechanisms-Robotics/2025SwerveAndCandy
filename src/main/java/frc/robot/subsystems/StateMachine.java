package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalOutput;

public class StateMachine extends SubsystemBase{

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

    private State myState;
    private DigitalOutput outputLEDWhite;
    private DigitalOutput outputLEDRed;
    private DigitalOutput outputLEDGreen;
    private DigitalOutput outputLEDBlue;

    /* OFFSEASON: setup data structures to manage the LED colors as an
    *    {R,G,B} array of booleans. Each possible state should then have
    *    a corresponding {R,G,B} array that represent the desired color
    *    for that state. THen when calling setState(), you don't need to hardcode
    *    the colors within the method, and don't need a separate method for
    *    each state. But rather can lookup the desired color for the newState
    *    and set the outputs based on the lookup. This will make the code more
    *    maintainable: easier to change color preferences, and easier to add new
    *    states in the future.
    */

    /** Constructor that takes an initial state as a parameter */
    public StateMachine(State inputState) {
        // map LEDs to correct ports
        outputLEDWhite = new DigitalOutput(0);
        outputLEDRed = new DigitalOutput(1);
        outputLEDGreen = new DigitalOutput(2);
        outputLEDBlue = new DigitalOutput(3);

        /* call method to set initial state and configure lights */
        setState(inputState);
    }

    /** default constructor with no parameter will
     * set initial state to "NoTargetIdentified"
     */
    public StateMachine() {
        /* Call other constructur and pass default value
         */
        this(State.NoTargetIdentified);
    }

    public State getState() {
        return myState;
    }

    public void setState(State newState) {
        // [M.Fox] This is not the most elegant way to do this,
        // but it is the most straightforward way to do for teaching
        // purposes. See comments at top of file labeled "OFFSEASON"
        // for a more maintanable way to do this.
        switch (newState) {
            case NoTargetIdentified:
                setNotIdentified();
                break;
            case DetectedReefTarget:
                setDetectedReefTarget();
                break;
            case DrivingToReefTarget:
                setDrivingToReefTarget();
                break;
            case ArrivedAtReefTarget:
                setArrivedAtReefTarget();
                break;
            default:
                break;
        }
        return;
    }

    private void setNotIdentified() {
        myState = State.NoTargetIdentified;
        
        // TODO: turn off all lights

        // TODO: turn on white light

        return;
    }

    private void setDetectedReefTarget() {
        myState = State.DetectedReefTarget;

        // TODO: turn off all lights
        
        // TODO: turn on green light

        return;
    }

    private void setDrivingToReefTarget() {
        myState = State.DrivingToReefTarget;

        // TODO: turn off all lights
        
        // TODO: turn on blue light

        return;
    }

    private void setArrivedAtReefTarget() {
        myState = State.ArrivedAtReefTarget;

        // TODO: turn off all lights
        
        // TODO: turn on red light
        
        return;
    }
}