package frc.robot.subsystems.coralmech;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Starting up and tuning procedure:
 * 1. Find the right motor ids using ![REV Hardware Client](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/rhc-1.7.3/REV-Hardware-Client-Setup-1.7.3.exe)
 * 2. Tune the intake and eject voltages
 * 3. Make sure inversions are done correctly, currently done in the methods rather than configuration
 */
public class CoralMech extends SubsystemBase {
    // We use REV-41-1600
    // Motor on the left of the coral mechanism, and the right of the coral mechanism, controlled by one spark
    private static final SparkMax m_motors = new SparkMax(20, MotorType.kBrushed); // TODO, find right id
    // not currently in use
    // Will be used to detect when a coral is in the mechanism, needed for knowing when to brake the motors
    private static final DigitalInput m_sensor = new DigitalInput(0);



    public CoralMech() {
        // Create motor configuration to apply to the wheel motors
        SparkMaxConfig config = new SparkMaxConfig();

        // kCoast might be worth trying but these wheels will be powering the motors holding the coral in place
        m_motors.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Runs the coral wheels at intakeVoltage in opposite directions
     */
    public void intakeCoral() {
        m_motors.getClosedLoopController().setReference(.6, ControlType.kDutyCycle);
        SmartDashboard.putString("CoralsMech/State", "intaking");
    }

    public void idle() {
        m_motors.getClosedLoopController().setReference(0, ControlType.kDutyCycle);
        SmartDashboard.putString("CoralMech/State", "idling");
    }

    @Override
    public void periodic() {
    }
}
