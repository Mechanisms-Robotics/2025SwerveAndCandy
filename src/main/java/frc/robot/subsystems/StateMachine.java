package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
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
    private static PWMSparkMax outputLEDRed;
    private static PWMSparkMax outputLEDGreen;
    private static PWMSparkMax outputLEDBlue;

    // brightness will be set based on speedvalues passed into PWM controller
    // ranging from 0 to 1
    private static final double MIN_BRIGHTNESS = 0;
    private static final double MAX_BRIGHTNESS = 0.83;

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

    /** Constructor */
    static {
        // map LEDs to correct ports
        outputLEDRed = new PWMSparkMax(6);
        outputLEDGreen = new PWMSparkMax(7);
        outputLEDBlue = new PWMSparkMax(8);

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
                setNotIdentified();
                break;
        }
        return;
    }

    private static void setNotIdentified() {
        myState = State.NoTargetIdentified;
        
        // turn on white light by driving all LEDs high
        white();

        return;
    }

    private static void setDetectedReefTarget() {
        myState = State.DetectedReefTarget;

        // turn on blue light 
        blue();

        return;
    }

    private static void setDrivingToReefTarget() {
        myState = State.DrivingToReefTarget;

        // turn on red light 
        red();

        return;
    }

    private static void setArrivedAtReefTarget() {
        myState = State.ArrivedAtReefTarget;

        // turn on green light 
        green();

        return;
    }

    public static void run() {
        SmartDashboard.putString("StateMachine/State", myState.name());
    }

    public static void red() {
        outputLEDRed.set(MAX_BRIGHTNESS);
        outputLEDGreen.set(MIN_BRIGHTNESS);
        outputLEDBlue.set(MIN_BRIGHTNESS);
    }

    public static void green() {
        outputLEDRed.set(MIN_BRIGHTNESS);
        outputLEDGreen.set(MAX_BRIGHTNESS);
        outputLEDBlue.set(MIN_BRIGHTNESS);
    }

    public static void blue() {
        outputLEDRed.set(MIN_BRIGHTNESS);
        outputLEDGreen.set(MIN_BRIGHTNESS);
        outputLEDBlue.set(MAX_BRIGHTNESS);
    }

    public static void white() {
        outputLEDRed.set(MAX_BRIGHTNESS);
        outputLEDGreen.set(MAX_BRIGHTNESS);
        outputLEDBlue.set(MAX_BRIGHTNESS);
    }
}