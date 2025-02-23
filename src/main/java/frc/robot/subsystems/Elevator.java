package frc.robot.subsystems;

/*

TUNING PROCEDURE

NOTE THAT I DIDN'T FINISH THIS, SO GO BACK THROUGH MONDAY AND REWORK BASED ON ABOVE
STARTING AROUND STEP 10.

1. Review all TODOs in the code and resolve the ones that matter.
2. Run this code on the robot to set leader and follower, etc.
3. Using the Rev hardware client, verify the leader ond follower
    configuration, especially the inversion of the follower. Power the
    leader at low power in the hardware client and see that the elevator
    moves up and down.
4. Determine the bottom and top soft limits using the Rev hardward client
    and then set the soft
    limits in this code based on that. Run that code one the RoboRIO to
    put those limits on the motor controllers.
5. Use the Rev hardware client to check that the soft limits stuck. Run
    it up and down to make sure it respects the soft limits.
6. Set MAX_ACCEL to something low like 1 RPM/s.
7. Set MAX_VEL to something like 60 RPM.
8. Set ALLOWED_ERROR to something like 50 (the encoder has 8192 ticks / revolution)
9. Set MAX_OUTPUT to 6 V.
10. Adjust KG until the elevator just starts to stuggle to lift.
11. Increase KV 

REFERENCES

Rev documentation on closed loop control
https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started

WPILib documentation on elevator tuning
https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-elevator.html#motion-profiled-feedforward-and-feedback-control

WIPLib documentation on trapezodial motion profiles
https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html

ElevatorFeedforward documentation
https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/controller/ElevatorFeedforward.html
*/

// TODO: Remove unused imports, commented out code, general cleanup...

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.subsystems.simulation.WPIElevator;

public class Elevator extends SubsystemBase {
    // TODO: Set soft limits
    // TODO: Limit switches (see getForwardLimitSwitch)
    // TODO: Zero encoder at init?
    // TODO: Put the motor in brake mode to prevent collapsing (maybe)

    // Elevator positions in encoder ticks
    // TODO: Determine experimentally and then write an equation for offline
    //   estimation of changes and put that here for posterity
    // TODO: We may need separate levels (or an offset) for algae vs. coral
    public final int RESTING = 0; // TODO make sure powered down completely when resting or very low to avoid power draw
    public final int PROCESSOR = 0;
    public final int LOADING = 0;
    public final int L1 = 0;
    public final int L2 = 0;
    public final int L3 = 0;
    public final int L4 = 0;
    public final int BARGE = 0;

    // TODO: Set CAN ids
    private final int LEADER_CAN_ID = 0;
    private final int FOLLOWER_CAN_ID = 0;

    // Motor controllers: one leader and one follower
    private final SparkMax m_leader;
    private final SparkMax m_follower;

    // The throughbore encoder is on the leader
    private final RelativeEncoder m_outputEncoder;

    // REV's built-in PID controller on the leader
    private final SparkClosedLoopController m_sparkClosedLoopController;

    // Tunables for the SparkMax PID and output
    private static final double KP = 0.0;
    private static final double KI = 0.0;
    private static final double KD = 0.0;
    private static final double MIN_OUTPUT = 0.0; // Volts
    private static final double MAX_OUTPUT = 0.0; // Volts
    
    // Tunables for the elevator feedforward
    private static final double KA = 0.0; // Acceleration feedforward Volts per something
    private static final double KS = 0.0; // Constant of static friction or whatever
    private static final double KG = 0.0; // Volts required to overcome gravity
    private static final double KV = 0.0; // Velocity constant in Volts per distance per second

    // Tunables for the elevator's trapezoidal motion profile
    // TODO: Decide on units and update the code below
    private static final double MAX_VELOCITY = 0.0; // I can probably put this in ticks
    private static final double MAX_ACCELERATION = 0.0; // Again, decide on units
    private static final double EPSILON = 0.0; // Allowed error, probably in ticks

    // TODO: This assumes the elevator starts at 0 position and 0 velocity, which may
    // need to be thunk about...
    private final TrapezoidProfile profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    private TrapezoidProfile.State m_setpoint  // The elevator's current position
        = new TrapezoidProfile.State(0.0, 0.0);
    private TrapezoidProfile.State m_goal = m_setpoint; // The elevator's goal setting

    public Elevator() {
        // Initialize the motor controllers
        m_leader = new SparkMax(LEADER_CAN_ID, MotorType.kBrushless);
        m_follower = new SparkMax(FOLLOWER_CAN_ID, MotorType.kBrushless);

        // Configure the motors

        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        leaderConfig.closedLoop
            .p(KP)
            .i(KI)
            .d(KD)
            .outputRange(MIN_OUTPUT, MAX_OUTPUT);

        followerConfig.follow(m_leader, true /* inverted */);

        m_leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Get the encoder from the leader (the throughbore encoder on the output shaft)
        m_outputEncoder = m_leader.getAlternateEncoder();

        // The onboard controller
        m_sparkClosedLoopController = m_leader.getClosedLoopController();
    }

    /**
     * Sets the desired elevator position.
     *
     * @param position The target position in TODO what units to use throughout?
     */
    public void setTargetPosition(double position) {
        // assumes final velocity is always zero
        m_goal = new TrapezoidProfile.State(position, 0.0);
    }

    /**
     * Returns the current elevator position.
     *
     * @return The current position in TODO units...
     */
    public double getCurrentPosition() {
        return m_outputEncoder.getPosition();
    }

    @Override
    public void periodic() {
        final double DT = 0.02; // seconds (based on periodic time)

        m_setpoint = profile.calculate(
            DT, // Time since last setpoint update
            m_setpoint, // Where we are in the current motion
            m_goal); // Where we want to be at the end of the motion
        
        if (Math.abs(m_setpoint.position - m_goal.position) > EPSILON) {
            // keep moving to the goal

            ElevatorFeedforward feedforward
                = new ElevatorFeedforward(KS, KG, KV, KA);
            
            m_sparkClosedLoopController.setReference(
                m_setpoint.position,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                feedforward.calculate(m_setpoint.velocity));
        }
    }
}
