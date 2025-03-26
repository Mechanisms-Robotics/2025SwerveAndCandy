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
    private DigitalOutput outputLEDGreen;
    private DigitalOutput outputLEDBlue;
    private DigitalOutput outputLEDRed;


    /** Constructor that takes an initial state as a parameter */
    public StateMachine(State initialState) {
        // map LEDs to correct ports
        outputLEDWhite = new DigitalOutput(0);
        outputLEDGreen = new DigitalOutput(1);
        outputLEDBlue = new DigitalOutput(2);
        outputLEDRed = new DigitalOutput(3);

        // turn on white light
        outputLEDWhite.set(true);

        /* call method to set initial state and configure lights */
        setState(initialState);
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