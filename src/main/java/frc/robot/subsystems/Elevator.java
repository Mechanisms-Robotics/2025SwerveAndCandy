package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.simulation.WPIElevator;

public class Elevator extends SubsystemBase {
    // TODO: Set soft limits
    // TODO: Limit switches (see getForwardLimitSwitch)

    // Elevator positions in encoder tics
    // TODO: Determine experimentally and then put an equation for offline estimation of changes and put that here for posterity
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

    // Elevator motion tunables TODO: Tune and clean up comments and all
    private static final double MAX_ACCEL = 0.0; // In RPM/s
    private static final double MAX_VEL = 0.0; // In RPM
    private static final double ALLOWED_ERROR = 0.0; // In encoder tics
    private static final double KP = 0.0;
    private static final double KI = 0.0;
    private static final double KD = 0.0;
    private static final double MIN_OUTPUT = 0.0; // Volts
    private static final double MAX_OUTPUT = 0.0; // Volts
    private static final double KA = 0.0; // Acceleration feedforward Volts per something
    private static final double KS = 0.0; // Constant of static friction or whatever
    private static final double KG = 0.0; // Volts required to overcome gravity
    private static final double KV = 0.0; // Velocity constant in Volts per distance per second

    /* TUNING PROCEDURE

    NOTE: This is going to change see https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html

    Alex says to use velocity control, not max motion. The motion profile will trapezodial and
    will happen in this code (see link above). When we get a new position setpoint, we create
    a trapezoidal profile using the WPILib class (you feed it current position and desired position).
    In the periodic, we sample the profile and get a desired velocity and acceleration based on the
    current position (or maybe time, more likely). We feed that into the Feedforward class (see below)
    and pass the desired velocity and the arbitrary feedforward to the SparkMax which is running
    and onboard velocity PID.

    Ref Motion Profiled, Feedforward, and Feedback Control
    at https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-elevator.html#motion-profiled-feedforward-and-feedback-control

    NOTE THAT I DIDN'T FINISH THIS, SO GO BACK THROUGH MONDAY AND REWORK BASED ON ABOVE
    STARTING AROUND STEP 10.

    1. Set the CAN IDs in this code
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
    8. Set ALLOWED_ERROR to something like 50 (the encoder has 8192 tics / revolution)
    9. Set MAX_OUTPUT to 6 V.
    10. Adjust KG until the elevator just starts to stuggle to lift.
    11. Increase KV 
    
     */

    /**
     * Constructs the Elevator subsystem.
     */
    public Elevator() {
        // Initialize motor controllers
        m_leader = new SparkMax(LEADER_CAN_ID, MotorType.kBrushless);
        m_follower = new SparkMax(FOLLOWER_CAN_ID, MotorType.kBrushless);

        // Configure the motors

        // TODO Get help / rework the control loop for good control and smoothness
        // Ref https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started
        // and see what we can do to tune and improve the control.

        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        leaderConfig.closedLoop.maxMotion
            .maxAcceleration(MAX_ACCEL)
            .maxVelocity(MAX_VEL)
            .allowedClosedLoopError(ALLOWED_ERROR);

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
     * @param position The target position
     */
    public void setTargetPosition(double position) {
        ElevatorFeedforward feedforward = new ElevatorFeedforward(
            KS, KG, KV, KA);

        // TODO: This ain't right
        // double ff = feedforward.calculate(m_sparkClosedLoopController.)

        // m_sparkClosedLoopController.setReference(
        //     position,
        //     ControlType.kMAXMotionPositionControl,
        //     ClosedLoopSlot.kSlot0, // TODO This allows multiple PID constants
        //     ff);
    }

    /**
     * Returns the current elevator position.
     *
     * @return The current position
     */
    public double getCurrentPosition() {
        return m_outputEncoder.getPosition();
    }

    @Override
    public void periodic() {
    }
}
