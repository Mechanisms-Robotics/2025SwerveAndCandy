package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeMech extends SubsystemBase {
    private TalonFX m_wristMotor = new TalonFX(0);
    private static final CANcoder canCoder = new CANcoder(12);
    // the left and right wheel motors are controlled by one spark
    private static final SparkMax m_wheelMotors = new SparkMax(21, MotorType.kBrushed);
    enum State {
      INTAKING,
      PLACING,
      IDLE
    }

    State state = State.INTAKING;


    public AlgaeMech() {
        SparkMaxConfig wheelMotorConfig = new SparkMaxConfig();

        m_wheelMotors.configure(wheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public void intake() {
        m_wheelMotors.getClosedLoopController().setReference(-.7, ControlType.kDutyCycle);
        state = State.INTAKING;
    }

    public void place() {
      m_wheelMotors.getClosedLoopController().setReference(1, ControlType.kDutyCycle);
      state = State.PLACING;
    }

    public void stop() {
      m_wheelMotors.getClosedLoopController().setReference(0, ControlType.kDutyCycle);
      state = State.IDLE;
    }

    public void toggleIntake() {
      if (state == State.INTAKING) {
        stop();
      } else {
        intake();
      }
    }

    @Override
    public void periodic() {
        // ranges from -.5-.5
        SmartDashboard.putNumber("AlgaeMech/Wrist/absolute position", canCoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("AlgaeMech/Wrist/position", canCoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("AlgaeMech/Wrist/position since boot", canCoder.getPositionSinceBoot().getValueAsDouble());
        SmartDashboard.putNumber("AlgaeMech/Wrist/angle", canCoder.getPositionSinceBoot().getValueAsDouble()%.5 * 2 * 360);
        SmartDashboard.putString("AlgaeMech/State", state.toString());
    }
}