package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralMech extends SubsystemBase {
    // We use REV-41-1600
    // Motor on the left of the coral mechanism, and the right of the coral mechanism, controlled by one spark
    private static final SparkMax m_motors = new SparkMax(20, MotorType.kBrushed);
    // not currently in use
    // Will be used to detect when a coral is in the mechanism, needed for knowing when to brake the motors
    private static final double feedDutyCycle = .6;
    /* DutyCycle equation: D = PW/T 
    Duty cycle is the ratio of the pulse width (active pulse time) over total time
    Further reading: https://en.wikipedia.org/wiki/Duty_cycle */

    enum State {
        FEEDING,
        IDLING
    }
    State state = State.IDLING;

    public CoralMech() {
        // Create motor configuration to apply to the wheel motors
        SparkMaxConfig config = new SparkMaxConfig();
        // TODO: if needed add configurations

        // kCoast might be worth trying but these wheels will be powering the motors holding the coral in place
        m_motors.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Spin the coral mech wheels so the coral moves out of the mechanism at intakeVoltage
     */
    public void feedCoral() {
        state = State.FEEDING;
        m_motors.getClosedLoopController().setReference(feedDutyCycle, ControlType.kDutyCycle);
        SmartDashboard.putString("CoralsMech/State", "intaking");
    }

    /**
     * Stop the coral mech wheel motors
     */
    public void idle() {
        state = State.IDLING;
        m_motors.getClosedLoopController().setReference(0, ControlType.kDutyCycle);
        SmartDashboard.putString("CoralMech/State", "idling");
    }

    /**
     * If it is not feeding, start feeding. Otherise, stop the coral wheels.
     */
    public void toggleFeed() {
        if (state == State.FEEDING) {
            idle();
        } else {
            feedCoral();
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("CoralMech/output current", m_motors.getOutputCurrent());
    }
}
