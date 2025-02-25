package frc.robot.subsystems;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;  

public class AlgaeMech extends SubsystemBase {
  private static final TalonFX m_wristMotor = new TalonFX(22);
  private final PIDController m_controller = new PIDController(0.0001, 0, 0);
  /* Angle of the arm on boot, rather what it should be on boot. When turning on the robot hold the arm up as far as it can go
  which should be an 86 degree anglr. */
  private static final double wristStartingAngle = 86;
  /* Angle the wrist id PIDing toward */
  private static double desiredAngle = wristStartingAngle;
  /* number of rotations of the motor per rotation of the hex shaft (note the hex shaft should never 
  make a full rotation because the arms of the algae mechanism cannot move like that)
  The falcon motor is attached to 2 5 to 1 gear boxes, a 18t pulley and a 27t pulley on a */
  private final double gearRatio = (27.0/18.0) * 25.0;
  // the left and right wheel motors are controlled by one spark
  private static final SparkMax m_wheelMotors = new SparkMax(21, MotorType.kBrushed);
  private static final double intakeDutyCycle = -.7;
  private static final double placingDutyCycle = 1;

  enum State {
    INTAKING,
    PLACING,
    IDLE
  }
  State state = State.INTAKING;

  public AlgaeMech() {
      SparkMaxConfig wheelMotorConfig = new SparkMaxConfig();
      SmartDashboard.putData("AlgaeMech/controler", m_controller);
      SmartDashboard.putNumber("AlgaeMech/Wrist/gear ratio", gearRatio);
      TalonFXConfiguration wristConfig = new TalonFXConfiguration();
      wristConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      // wristConfig.Slot0.kG = 0;
      // wristConfig.Slot0.kV = 0;
      // wristConfig.Slot0.kP = 0;
      // wristConfig.Slot0.kI = 0;
      // wristConfig.Slot0.kD = 0;
      m_wristMotor.getConfigurator().apply(wristConfig);
      m_wheelMotors.configure(wheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Spin the wheels toward the robot using DutyCycle to intake the Algae
   */
  public void intake() {
      m_wheelMotors.getClosedLoopController().setReference(intakeDutyCycle, ControlType.kDutyCycle);
      state = State.INTAKING;
  }

  /**
   * Spin the wheels away from the robot to place algae
   */
  public void place() {
    m_wheelMotors.getClosedLoopController().setReference(placingDutyCycle, ControlType.kDutyCycle);
    state = State.PLACING;
  }

  /**
   * Stop the motors
   */
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

  /**
   * Outputs the angle of the arm based on the internal encoder of the wrist motor
   * @return
   */
  public double getWristAngle() {
    return m_wristMotor.getPosition().getValueAsDouble() * 360 / gearRatio + wristStartingAngle;
  }

  /**
   * Get velocity in degrees per seconds
   * 
   * @return velocity in degrees per second
   */
  public double getWristVelocity() {
    return m_wristMotor.getVelocity().getValueAsDouble() * 360 / gearRatio;
  }

  /**
   * Start piding to an angle
   * 
   * @param angle angle to pid to
   */
  public void setWristAngle(double angle) {
    this.desiredAngle = angle;
  }

  

  @Override
  public void periodic() {
    m_wristMotor.set(m_controller.calculate(getWristAngle(), desiredAngle));
    // ranges from -.5-.5
    SmartDashboard.putString("AlgaeMech/State", state.toString());
    SmartDashboard.putNumber("AlgaeMech/Wrist/Angle", getWristAngle());
    SmartDashboard.putNumber("AlgaeMech/Wrist/Velocity (degrees per seconds)", getWristVelocity());
    SmartDashboard.putNumber("AlgaeMech/wrist motor/velocity", m_wristMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("AlgaeMech/wrist motor/position", m_wristMotor.getPosition().getValueAsDouble());
  }
}