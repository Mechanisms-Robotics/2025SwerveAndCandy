// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.AlgaeMech;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.StateMachine;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  // These maps map elevator positions in ticks to swerve maximum velocities
  private final InterpolatingDoubleTreeMap swerveVelocityMap = new InterpolatingDoubleTreeMap(); // meters per second
  private final InterpolatingDoubleTreeMap swerveRotationSpeedMap = new InterpolatingDoubleTreeMap(); // radians per second

  /**
   * This should be called in every init to make sure that if the robot is disabled or re-enabled or
   * transitions from auto to teleop, nothing should move. The odometery and zero positions should NOT
   * be reset execpt when the robot code starts.
   */
  public void resetMotorsOnInit() {
    m_robotContainer.m_algaeMech.setWristBrake(true);
    m_robotContainer.m_algaeMech.setWristAngle(AlgaeMech.WRIST_ANGLE_UP);
    // m_robotContainer.m_algaeMech.setWristAngle(
    //   m_robotContainer.m_algaeMech.getWristAngle()
    // );

    m_robotContainer.setElevatorToWhereItsAt();

    m_robotContainer.m_coralMech.stop();
  }

  public Robot()
  {
    instance = this;

    if (!isSimulation()) {
      // Camera setup
      // Access the camera's web server at http://10.87.36.2:1181/
      // From there you can see the allowed resolutions and frame rates, etc.
  
      final int CAMERA_RESOLUTION_W = 640;
      final int CAMERA_RESOLUTION_H = 480;
      final int CAMERA_FRAME_RATE = 30;
  
      UsbCamera camera = CameraServer.startAutomaticCapture();
      camera.setResolution(CAMERA_RESOLUTION_W, CAMERA_RESOLUTION_H);
      camera.setFPS(CAMERA_FRAME_RATE);
    }
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
            
    resetMotorsOnInit();
  }
          
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    
    // Gets the current position of the elevator and uses the maps to reduce the speed of the swerve
    
    final double MIN_SPEED_REDUCTION_FACTOR = 0.15;
    double currentPosition = m_robotContainer.m_elevator.getCurrentPosition();
    double reductionFactor = 1.0 + (MIN_SPEED_REDUCTION_FACTOR - 1.0)
      *(currentPosition - Elevator.RESTING)/(Elevator.BARGE - Elevator.RESTING);
    double swerveVelocityThrottle = Constants.MAX_SPEED*reductionFactor;
    double swerveRotationThrottle = m_robotContainer.m_drivebase.defaultAngularVelocity*reductionFactor;
  
    SmartDashboard.putNumber("Swerve/Velocity Throttle", swerveVelocityThrottle);
    SmartDashboard.putNumber("Swerve/Rotation Throttle", swerveRotationThrottle);
    
    m_robotContainer.m_drivebase.setMaxSpeed(swerveVelocityThrottle, swerveRotationThrottle);

    SmartDashboard.putData("Command Scheduler Visualisation", CommandScheduler.getInstance());
    CommandScheduler.getInstance().run();
    StateMachine.run();
    outputRobotPose();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    m_robotContainer.setElevatorToWhereItsAt(); // prevent elevator from moving
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_AND_ALGAE_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      m_robotContainer.m_algaeMech.setWristBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    resetMotorsOnInit();
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    resetMotorsOnInit();

    // // This makes sure that the autonomous stops running when
    // // teleop starts running. If you want the autonomous to
    // // continue until interrupted by another command, remove
    // // this line or comment it out.
    m_robotContainer.setMotorBrake(true);
    
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {

  }

  @Override
  public void testInit()
  {
    resetMotorsOnInit();
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
    resetMotorsOnInit();
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }


  public void outputRobotPose()
  {
    Pose2d robotPose = m_robotContainer.m_drivebase.getPose();
    SmartDashboard.putNumber("Pose X: ", robotPose.getTranslation().getX());
    SmartDashboard.putNumber("Pose Y: ", robotPose.getTranslation().getY()); 
    SmartDashboard.putNumber("Pose Theta: ", robotPose.getRotation().getDegrees());
  }
}
