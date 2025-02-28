package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;  

public class AlgaeMech extends SubsystemBase {

  public final static int WRIST_MOTOR_CAN_ID = 22;

  private static final TalonFX m_wristMotor = new TalonFX(WRIST_MOTOR_CAN_ID);

  private final static double P = 0.01;
  private final static double I = 0.0;
  private final static double D = 0.0;

  private final PIDController m_controller = new PIDController(P, I, D);
  
  /* Angle of the arm on boot, rather what it should be on boot. When turning on the
  robot hold the arm up as far as it can go which should be an 86 degree angle.
  
  Zero degrees is pointed straight out. */

  public final static double WRIST_STARTING_CONFIGURATION_ANGLE = 86.0; // Degrees
  public final static double WRIST_ALGAE_PICKUP_ANGLE = -5.0; // UNUSED RIGHT NOW (26 FEB)
  private final double m_wristStartingAngle; // Degrees

  /* Angle the wrist id PIDing toward */
  private double m_desiredAngle;

  /* number of rotations of the motor per rotation of the hex shaft (note the hex shaft should never 
  make a full rotation because the arms of the algae mechanism cannot move like that)
  The falcon motor is attached to 2 5 to 1 gear boxes, a 18t pulley and a 27t pulley on a */
  private final double GEAR_RATIO = (27.0/18.0) * 25.0;

  // the left and right wheel motors are controlled by one spark
  private static final int ALGAE_INTAKE_MOTORS_CAN_ID = 21;
  private static final SparkMax m_wheelMotors = new SparkMax(
    ALGAE_INTAKE_MOTORS_CAN_ID, MotorType.kBrushed);

  private static final double INTAKE_DUTY_CYCLE = -0.7;
  private static final double OUTTAKING_DUTY_CYCLE = 1.0;

  private static final double WRIST_MOTOR_CURRENT_LIMIT = 20.0; // A

  enum State {
    /** Motors are spinning inward, so it can intake algae */
    INTAKING,
    /** Motors are spinning outward, so it can outtake algae into the processor or toss it onto the barge */
    OUTTAKING,
    /** Motors are not moving */
    STOPPED
  }

  private State state = State.STOPPED;

  public AlgaeMech() {
      SmartDashboard.putData("AlgaeMech/controler", m_controller);
      SmartDashboard.putNumber("AlgaeMech/Wrist/gear ratio", GEAR_RATIO);

      TalonFXConfiguration wristConfig = new TalonFXConfiguration();
      wristConfig.CurrentLimits.StatorCurrentLimit = WRIST_MOTOR_CURRENT_LIMIT;
      wristConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      wristConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      m_wristMotor.getConfigurator().apply(wristConfig);
      setWristBrake(true);

      //SparkMaxConfig wheelMotorConfig = new SparkMaxConfig();
      //m_wheelMotors.configure(wheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 
      m_wristStartingAngle = m_wristMotor.getPosition().getValueAsDouble() * 360.0 / GEAR_RATIO;
      m_desiredAngle = m_wristStartingAngle;
    }

    public void setWristBrake(boolean brake) {
      m_wristMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

  /**
   * Spin the wheels toward the robot using DutyCycle to intake the Algae
   */
  public void intake() {
      m_wheelMotors.getClosedLoopController().setReference(
        INTAKE_DUTY_CYCLE, ControlType.kDutyCycle);
      state = State.INTAKING;
  }

  /**
   * Spin the wheels away from the robot to place algae
   */
  public void outtake() {
    m_wheelMotors.getClosedLoopController().setReference(
      OUTTAKING_DUTY_CYCLE, ControlType.kDutyCycle);
    state = State.OUTTAKING;
  }

  /**
   * Stop the wheels motors
   */
  public void stop() {
    m_wheelMotors.getClosedLoopController().setReference(
      0, ControlType.kDutyCycle);
    state = State.STOPPED;
  }

  /**
   * If it is not (i.e. in any other state other than), intaking, start intaknig.
   * Otherwise, if it is in intaking, stop the wheels motors
   */
  public void toggleIntake() {
    if (state == State.INTAKING) {
      stop();
    } else {
      intake();
    }
  }

  /**
   * If it is not (i.e. in any other state other than), placing, start placing.
   * Otherwise, if it is in placing, stop the wheels from spinning
   */
  public void toggleOuttake() {
    if (state == State.OUTTAKING) {
      stop();
    } else {
      outtake();
    }
  }

  /**
   * Outputs the angle of the arm based on the internal encoder of the wrist motor
   * @return
   */
  public double getWristAngle() {
    return m_wristMotor.getPosition().getValueAsDouble() * 360.0 / GEAR_RATIO
      + WRIST_STARTING_CONFIGURATION_ANGLE - m_wristStartingAngle;
  }

  /**
   * Get velocity in degrees per seconds
   * 
   * @return velocity in degrees per second
   */
  public double getWristVelocity() {
    return m_wristMotor.getVelocity().getValueAsDouble() * 360.0 / GEAR_RATIO;
  }

  /**
   * Start piding to an angle
   * 
   * @param angle angle to pid to (degrees)
   */
  public void setWristAngle(double angle) {
    this.m_desiredAngle = angle;
  }

  @Override
  public void periodic() {
    // output = P*(wrist angle - desired angle)
    // So if maximum error is 100 degrees, a P = 0.01 would yield an output of 1.0

    double output = m_controller.calculate(getWristAngle(), m_desiredAngle);
    if (output < 0) {
      final double POOR_MANS_FEEDFORWARD = 5.0;
      output /= POOR_MANS_FEEDFORWARD;
    }
    m_wristMotor.set(output);

    SmartDashboard.putNumber("AlgaeMech/Wrist/PID Output", output);
    SmartDashboard.putString("AlgaeMech/State", state.toString());
    SmartDashboard.putNumber("AlgaeMech/Wrist/Desired Angle", m_desiredAngle);
    SmartDashboard.putNumber("AlgaeMech/Wrist/Angle", getWristAngle());
    SmartDashboard.putNumber("AlgaeMech/Wrist/Velocity (degrees per seconds)", getWristVelocity());
    SmartDashboard.putNumber("AlgaeMech/wrist motor/velocity", m_wristMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("AlgaeMech/wrist motor/position", m_wristMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("AlgaeMech/Wrist/OutputCurrent", m_wristMotor.getStatorCurrent().getValueAsDouble());
  }
}