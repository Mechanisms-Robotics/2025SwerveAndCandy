package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.RelativeEncoder;

//import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*

STARTUP AND TUNING PROCEDURE

We deviated from these instructions a bit. Use some common sense.

0. Verify that the throughbore encoder switch is on A and the breakout board
    is on ABS PWM.
1. Set the motor CAN IDs in the code. Use the Rev Hardware client to check them,
    if necessary.
2. Run this code on the robot to set leader and follower, soft limits, etc.
3. Using the Rev hardware client, verify the leader ond follower
    configuration. Verify the soft limits are set and enabled. Power the leader at
    low power in the hardware client and see that the elevator moves up and down
    and respects the limits. Verify that the encoder is zeroed and that the limits
    are respected.
4. Determine the bottom and top soft limits using the Rev hardware client
    and then set the new soft limits in this code based on that. Run that code
    once on the RoboRIO to put those limits on the motor controllers.
5. Use the Rev hardware client to check that the soft limits stuck. Run
    it up and down to make sure it respects the soft limits.
6. Put in reasonable numbers for MAX_VELOCITY, MAX_ACCELERATION, and EPSILON
    on the trapezoidal profile. Note that I've mapped square to L1 and circle to
    RESTING for testing.
7. Increase KG until the elevator is "weightless" (just starting to move up).
    This may take two decimal places of precision.
8. Increase the velocity feedforward gain (KV) until the straight segments of the
    elevator actual motion have the same slope as the desired motion.
9. Increase the velocity feedforward gain until the straight segments of the elevator
    actual motion have the same slope as the desired motion.
10. Increase KP until the actual position starts to overshoot the target, then back
    it off by 20%.
11. Keep tuning... Somehow...

REFERENCES

Rev documentation on closed loop control
https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started

WPILib documentation on elevator tuning
https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-elevator.html#motion-profiled-feedforward-and-feedback-control

WIPLib documentation on trapezodial motion profiles
https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html

ElevatorFeedforward documentation
https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/controller/ElevatorFeedforward.html

Chief Delphi thread on througbore as an alternate encoder
https://www.chiefdelphi.com/t/rev-through-bore-encoder-not-working-in-alternate-mode/491304

That same team's elevator code
https://github.com/FRC3546/2025-Reefscape-Competition/blob/main/src/main/java/frc/robot/subsystems/ElevatorSubsystem.java
*/

public class Elevator extends SubsystemBase {
    // Soft limits determined experimentally. We set the lower limit at 100, at
    // which time gravity will let it fall a little lower.
    private static final double FORWARD_SOFT_LIMIT = 40500.0; // Ticks
    private static final double REVERSE_SOFT_LIMIT = 10.0; // Ticks

    // Elevator positions in encoder ticks (8,192 ticks per revolution)
    // about 700 ticks per inch

    public static final int RESTING = (int)REVERSE_SOFT_LIMIT;
    public static final int PROCESSOR = 500;
    public static final int LOADING = 1000;
    public static final int L1 = 6000;
    public static int L1_Offset = 6000;
    public static final int L2 = 12500;
    public static int L2_Offset = 12500;
    public static int L2_ALGAE_OFFSET = 16500;
    public static final int L3 = 19700;
    public static int L3_Offset = 19700;
    public static int L3_ALGAE_OFFSET = 23500;
    public static final int L4 = 32700;
    public static int L4_Offset = 34000;
    public static final int BARGE = (int)FORWARD_SOFT_LIMIT - 100;

    private static final int LEADER_CAN_ID = 11;
    private static final int FOLLOWER_CAN_ID = 10;

    // Motor controllers: one leader and one follower
    private final SparkMax m_leader = new SparkMax(LEADER_CAN_ID, MotorType.kBrushless);
    private final SparkMax m_follower = new SparkMax(FOLLOWER_CAN_ID, MotorType.kBrushless);

    // The throughbore encoder is on the leader
    private RelativeEncoder m_outputEncoder = m_leader.getAlternateEncoder();

    // REV's built-in PID controller on the leader
    private final SparkClosedLoopController m_sparkClosedLoopController
        = m_leader.getClosedLoopController();

    // Tunables for the SparkMax PID and output
    private static final double KP = 0.000175;
    private static final double KI = 0.0;
    private static final double KD = 0.0;
    private static final double MIN_OUTPUT = -10.0; // Volts
    private static final double MAX_OUTPUT = 10.0; // Volts
    
    // Tunables for the elevator feedforward
    // private static final double KA = 0.0; // Acceleration feedforward Volts per something
    // private static final double KS = 0.0; // Constant of static friction or whatever
    // private static final double KG = 0.0; // Volts required to overcome gravity
    // private static final double KV = 0.0; // Velocity constant in Volts per distance per second

    // Tunables for the elevator's trapezoidal motion profile
    private static final double MAX_VELOCITY = 5*8192.0; // Ticks per second?
    private static final double MAX_ACCELERATION = 8*8192; // Ticks per second per second?
    private static final double EPSILON = 5.0; // Allowed error, presumably in ticks

    private final TrapezoidProfile profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    private TrapezoidProfile.State m_setpoint  // The elevator's current position
        = new TrapezoidProfile.State(0.0, 0.0); // Starting lowered
    private TrapezoidProfile.State m_goal = m_setpoint; // The elevator's goal setting

    private static final int CURRENT_LIMIT = 40; // Amps per motor

    public Elevator() {
        // Zero the encoder
        m_outputEncoder.setPosition(0.0);

        // Configure the motors

        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        final int THROUGHBORE_TICKS_PER_REVOLUTION = 8192;
        leaderConfig.alternateEncoder
            .countsPerRevolution(THROUGHBORE_TICKS_PER_REVOLUTION)
            .setSparkMaxDataPortConfig()
            .positionConversionFactor(THROUGHBORE_TICKS_PER_REVOLUTION)
            .velocityConversionFactor(THROUGHBORE_TICKS_PER_REVOLUTION)
            .inverted(true);

        leaderConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(CURRENT_LIMIT);
        
        followerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(CURRENT_LIMIT);

        leaderConfig.softLimit
            .forwardSoftLimit(FORWARD_SOFT_LIMIT)
            .forwardSoftLimitEnabled(true);
        leaderConfig.softLimit
            .reverseSoftLimit(REVERSE_SOFT_LIMIT)
            .reverseSoftLimitEnabled(true);

        leaderConfig.closedLoop
            .p(KP)
            .i(KI)
            .d(KD)
            .outputRange(MIN_OUTPUT, MAX_OUTPUT)
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);

        followerConfig.follow(m_leader, false /* NOT inverted */);

        m_leader.configure(
            leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_follower.configure(
            followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the desired elevator position.
     *
     * @param position The target position in ticks.
     */
    public void setTargetPosition(double position) {
        // assumes final velocity is always zero
        m_goal = new TrapezoidProfile.State(position, 0.0);
    }

    /**
     * Returns the current elevator position.
     *
     * @return The current position in ticks.
     */
    public double getCurrentPosition() {
        return m_outputEncoder.getPosition();
    }

    public void increaseL2AlgaeOffset(double ticks) {
        L2_ALGAE_OFFSET += ticks;
    }

    public void increaseL2Offset(double ticks) {
        L2_Offset += ticks;
    }

    public void increaseL3AlgaeOffset(double ticks) {
        L3_ALGAE_OFFSET += ticks;
    }

    public void increaseL3Offset(double ticks) {
        L3_Offset += ticks;
    }

    public void increaseL4Offset(double ticks) {
        L4_Offset += ticks;
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/Position", getCurrentPosition());
        SmartDashboard.putNumber("Elevator/Goal ", m_goal.position);
        SmartDashboard.putNumber("Elevator/Motor leader/Current", m_leader.getOutputCurrent());
        SmartDashboard.putNumber("Elevator/Motor leader/Temperature", m_leader.getMotorTemperature());
        SmartDashboard.putNumber("Elevator/Motor follower/Current", m_follower.getOutputCurrent());
        SmartDashboard.putNumber("Elevator/Motor follower/Temperature", m_follower.getMotorTemperature());
        SmartDashboard.putNumber("Elevator/Bus Voltage", m_leader.getBusVoltage());

        final double DT = 0.02; // seconds (based on periodic time)

        m_setpoint = profile.calculate(
            DT, // Time since last setpoint update
            m_setpoint, // Where we are in the current motion
            m_goal); // Where we want to be at the end of the motion
        
        if (Math.abs(m_setpoint.position - m_goal.position) > EPSILON) {
            // keep moving to the goal

            // ElevatorFeedforward feedforward
            //     = new ElevatorFeedforward(KS, KG, KV, KA);
            // double ff = feedforward.calculate(m_setpoint.velocity);


            // move down faster  TODO: we can do better than this
            final double DOWNWARD_FF_HACK = 0.0; // Volts
            final double DOWNWARD_FF_HACK_EPSILON = L1/2; // ticks
            
            double ff = (m_setpoint.position - m_goal.position) > DOWNWARD_FF_HACK_EPSILON
                ? DOWNWARD_FF_HACK : 0.0;
            
            m_sparkClosedLoopController.setReference(
                m_setpoint.position,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                ff);
        }
    }
}
