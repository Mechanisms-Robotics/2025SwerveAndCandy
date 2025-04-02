package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StateMachine {

    public static enum State {
        /**No AprilTag seen by the robot */
        NoTargetIdentified,
        /**AprilTag seen on the reef */
        DetectedReefTarget,
        /**Going towards AprilTag on the reef */
        DrivingToReefTarget,
        /**Arrived at AprilTag on the reef */
        ArrivedAtReefTarget
    }

    private static State myState;
    private static DigitalOutput outputLEDWhite;
    private static DigitalOutput outputLEDRed;
    private static DigitalOutput outputLEDGreen;
    private static DigitalOutput outputLEDBlue;

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
    static {
        // map LEDs to correct ports
        outputLEDRed = new DigitalOutput(6);
        outputLEDGreen = new DigitalOutput(7);
        outputLEDBlue = new DigitalOutput(8);

        /* call method to set initial state and configure lights */
        setState(State.NoTargetIdentified);
    }

    public static State getState() {
        return myState;
    }

    public static void setState(State newState) {
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

    private static void setNotIdentified() {
        myState = State.NoTargetIdentified;
        
        // turn on white light 
        outputLEDRed.set(true);
        outputLEDGreen.set(true);
        outputLEDBlue.set(true);
        

        return;
    }

    private static void setDetectedReefTarget() {
        myState = State.DetectedReefTarget;

        // turn on blue light 
        outputLEDRed.set(false);
        outputLEDGreen.set(false);
        outputLEDBlue.set(true);


        return;
    }

    private static void setDrivingToReefTarget() {
        myState = State.DrivingToReefTarget;

        // turn on red light 
        outputLEDRed.set(true);
        outputLEDGreen.set(false);
        outputLEDBlue.set(false);


        return;
    }

    private static void setArrivedAtReefTarget() {
        myState = State.ArrivedAtReefTarget;

        // turn on green light 
        outputLEDRed.set(false);
        outputLEDGreen.set(true);
        outputLEDBlue.set(false);

        
        return;
    }


    public static void run() {
        SmartDashboard.putString("StateMachine/State", myState.name());
    }
}