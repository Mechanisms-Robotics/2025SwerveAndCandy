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

public class CoralMech extends SubsystemBase {
    // We use REV-41-1600
    // Motor on the left of the coral mechanism, and the right of the coral mechanism
    private static final SparkMax motorL = new SparkMax(0, MotorType.kBrushed);
    private static final SparkMax motorR = new SparkMax(1, MotorType.kBrushed);
    // not currently in use
    private static final DigitalInput sensor = new DigitalInput(0);

    private static final double ejectVoltage = 1;
    private static final double intakeVoltage = 1;


    public CoralMech() {
        // motor configuration
        SparkMaxConfig config = new SparkMaxConfig();
        // kCoast might be worth trying but these wheels will be powering the motors holding the coral in place
        config.idleMode(IdleMode.kBrake);
        motorL.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorR.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SmartDashboard.putNumber("CoralMech/ejectVoltage", ejectVoltage);
    }

    public void placeCoral() {
        motorL.setVoltage(ejectVoltage);
        motorR.setVoltage(-ejectVoltage);
        SmartDashboard.putString("CoralMech/State", "placing");
    }

    public void intakeCoral() {
        motorL.setVoltage(intakeVoltage);
        motorR.setVoltage(-intakeVoltage);
        SmartDashboard.putString("CoralsMech/State", "intaking");
    }

    @Override
    public void periodic() {
    }
}
