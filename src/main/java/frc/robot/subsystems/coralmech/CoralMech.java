package frc.robot.subsystems.coralmech;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
    // Motor on the left of the coral mechanism, and the right of the coral mechanism
    private static final SparkMax m_motorL = new SparkMax(0, MotorType.kBrushed); // TODO, find right id
    private static final SparkMax m_motorR = new SparkMax(1, MotorType.kBrushed); // TODO, find right id
    // not currently in use
    // Will be used to detect when a coral is in the mechanism, needed for knowing when to brake the motors
    private static final DigitalInput m_sensor = new DigitalInput(0);

    private static final double ejectVoltage = 1; // TODO: tune
    private static final double intakeVoltage = 1; // TODO: tune


    public CoralMech() {
        // Create motor configuration to apply to the wheel motors
        SparkMaxConfig config = new SparkMaxConfig();
        // kCoast might be worth trying but these wheels will be powering the motors holding the coral in place
        config.idleMode(IdleMode.kBrake);
        m_motorL.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_motorR.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        SmartDashboard.putNumber("CoralMech/ejectVoltage", ejectVoltage);
    }

    /**
     * Runs the coral wheel motors at ejectVoltage in opposite directions
     */
    public void placeCoral() {
        m_motorL.setVoltage(ejectVoltage);
        m_motorR.setVoltage(-ejectVoltage);
        SmartDashboard.putString("CoralMech/State", "placing");
    }

    /**
     * Runs the coral wheels at intakeVoltage in opposite directions
     */
    public void intakeCoral() {
        m_motorL.setVoltage(intakeVoltage);
        m_motorR.setVoltage(-intakeVoltage);
        SmartDashboard.putString("CoralsMech/State", "intaking");
    }

    @Override
    public void periodic() {
    }
}
