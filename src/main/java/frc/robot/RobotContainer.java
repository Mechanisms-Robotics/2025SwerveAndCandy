// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.L2;
import frc.robot.commands.L3;
import frc.robot.commands.autos.TimedLeave;
import frc.robot.subsystems.AlgaeMech;
import frc.robot.subsystems.CoralMech;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.Supplier;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandPS4Controller driverController = new CommandPS4Controller(0);
  private final CommandPS4Controller secondaryController = new CommandPS4Controller(3);
  private final Joystick shifter = new Joystick(1);
  private final Joystick gamePad = new Joystick(2);

  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
    "swerve"));
  public final Elevator m_elevator = new Elevator();
  public final CoralMech m_coralMech = new CoralMech();
  public final AlgaeMech m_algaeMech = new AlgaeMech();
  private final SendableChooser<Command> m_autoChooser = new SendableChooser();

  public void setElevatorToWhereItsAt() {
    // Prevents the elevator from moving on enable
    m_elevator.setTargetPosition(m_elevator.getCurrentPosition());
  }

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   * 
   * I THINK THIS IS WHERE WE CAN CHANGE THE DRIVER CONTROL FEEL -- JVO 2 MARCH
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
        () -> Math.pow(driverController.getLeftY(), 3),
        () -> Math.pow(driverController.getLeftX(), 3))
    .withControllerRotationAxis(() -> -Math.signum(driverController.getRightX())*Math.pow(driverController.getRightX(), 2))
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverController::getRightX,
                                                                                             driverController::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
                                                                        () -> -driverController.getLeftY(),
                                                                        () -> -driverController.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverController.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    createAutos();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedAnglularVelocity = m_drivebase.driveFieldOriented(driveAngularVelocity);

    if (RobotBase.isSimulation())
    {
      m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    } else
    {
      m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      driveDirectAngleKeyboard.driveToPose(() -> new Pose2d(new Translation2d(9, 3),
                                                            Rotation2d.fromDegrees(90)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5,
                                                                                     3)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(
                                                                         Math.toRadians(
                                                                             360),
                                                                         Math.toRadians(
                                                                             90))));
      driverController.options().onTrue(Commands.runOnce(() -> m_drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverController.button(1).whileTrue(m_drivebase.sysIdDriveMotorCommand());
      driverController.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

      // Joystick buttons = new Joystick(0);
      // Supplier<Boolean> clutch = () -> buttons.getRawButton(4);
      // new Trigger(() -> buttons.getRawButtonPressed(1)).onTrue(new L2(m_elevator, m_algaeMech, clutch, secondaryController));
      // new Trigger(() -> buttons.getRawButtonPressed(2)).onTrue(new L3(m_elevator, m_algaeMech, clutch));
    }
    if (DriverStation.isTest())
    {
      m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverController.square().whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly());
      driverController.triangle().whileTrue(m_drivebase.driveToDistanceCommand(1.0, 0.2));
      driverController.options().onTrue((Commands.runOnce(m_drivebase::zeroGyro)));
      driverController.share().whileTrue(m_drivebase.centerModulesCommand());
      driverController.L2().onTrue(Commands.none());
      driverController.R2().onTrue(Commands.none());
    } else
    // ******** The following code is the acutal drive code used *******
    {
      driverController.cross().onTrue(Commands.runOnce(m_drivebase::zeroGyro));

      // Algae Mech
      driverController.L2().onTrue(Commands.runOnce(m_algaeMech::intake));
      driverController.L2().onFalse(Commands.runOnce(m_algaeMech::stop));
      driverController.R2().onTrue(Commands.runOnce(m_algaeMech::toggleOuttake));
      driverController.R2().onFalse(Commands.runOnce(m_algaeMech::stop));

      driverController.square().onTrue(Commands.runOnce(
        () -> m_algaeMech.setWristAngle(AlgaeMech.WRIST_ANGLE_DOWN)));

      driverController.triangle().onTrue(Commands.runOnce(
        () -> m_algaeMech.setWristAngle(AlgaeMech.WRIST_ANGLE_LEVEL)));
  
      driverController.circle().onTrue(Commands.runOnce(
        () -> m_algaeMech.setWristAngle(AlgaeMech.WRIST_ANGLE_UP)));

      // Coral Mech
      driverController.R1().onTrue(Commands.runOnce(m_coralMech::feed, m_coralMech));
      driverController.R1().onFalse(Commands.runOnce(m_coralMech::stop, m_coralMech));
      
      // Elevator
      // Move elevator up and down manualy, kept here for now. I have no particular commitment to keeping these here
      // May be removed if at all needed. - Micah
      driverController.povDown().onTrue(Commands.runOnce(
        () -> m_elevator.setTargetPosition(m_elevator.getCurrentPosition() - 500), m_elevator)
      );
        
      driverController.povUp().onTrue(Commands.runOnce(
        () -> m_elevator.setTargetPosition(m_elevator.getCurrentPosition() + 1000), m_elevator)
      );

      // Modifier controlls for elevator positions
      Supplier<Boolean> clutch = () -> secondaryController.touchpad().getAsBoolean() || gamePad.getRawButton(1);
      Supplier<Boolean> up = () -> secondaryController.povUp().getAsBoolean();
      Supplier<Boolean> down = () -> secondaryController.povDown().getAsBoolean();
      secondaryController.cross().onTrue(Commands.runOnce(
        () -> m_elevator.setTargetPosition(Elevator.L1), m_elevator));

      secondaryController.square().onTrue(new L2(m_elevator, m_algaeMech, clutch, up, down));
      secondaryController.circle().onTrue(new L3(m_elevator, m_algaeMech, clutch, up, down));

      secondaryController.triangle().onTrue(Commands.runOnce(
        () -> m_elevator.setTargetPosition(Elevator.L4), m_elevator));
  
      secondaryController.R1().onTrue(Commands.runOnce(
        () -> m_elevator.setTargetPosition(Elevator.BARGE), m_elevator)
      );

      secondaryController.povRight().whileTrue(
        Commands.run(() -> m_algaeMech.bumpWristUp(AlgaeMech.WRIST_BUMP)));

      secondaryController.povLeft().whileTrue(
        Commands.run(() -> m_algaeMech.bumpWristUp(-AlgaeMech.WRIST_BUMP)));

      secondaryController.L1().onTrue(Commands.runOnce(
        () -> m_elevator.setTargetPosition(Elevator.RESTING), m_elevator)
      );
  
      new Trigger(() -> shifter.getRawButtonPressed(1)).onTrue(Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.L1), m_elevator));
      // uncomment these lines to sue the commands that automatically lower the algae arms to pick up algae when in clutch
      // and comment the lines that would cause problems (.getRawButton(2, 3))
      // new Trigger(() -> shifter.getRawButton(2)).whileTrue(new L2(m_elevator, m_algaeMech, clutch));
      // new Trigger(() -> shifter.getRawButton(3)).whileTrue(new L3(m_elevator, m_algaeMech, clutch));
      // Since these have two different modes, they need to be triggered continnously to update the mode in the event the clutch is engaged
      new Trigger(() -> shifter.getRawButton(2)).whileTrue(new L2(m_elevator, m_algaeMech, clutch, up, down));
      new Trigger(() -> shifter.getRawButton(3)).whileTrue(new L3(m_elevator, m_algaeMech, clutch, up, down));
      new Trigger(() -> shifter.getRawButton(4)).whileTrue(
        Commands.run(() -> m_elevator.setTargetPosition(clutch.get() ? Elevator.L4_OFFSET : Elevator.L4), m_elevator));
      new Trigger(() -> shifter.getRawButtonPressed(6)).onTrue(Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.PROCESSOR), m_elevator));
      new Trigger(() -> shifter.getRawButtonPressed(7)).onTrue(Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.BARGE), m_elevator));
      new Trigger(() -> shifter.getRawButtonPressed(8)).onTrue(Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.RESTING), m_elevator));
        // new Trigger(() -> shifter.getRawButton(1) || shifter.getRawButton(2)
      // || shifter.getRawButton(3) || shifter.getRawButton(4)).onFalse(
        //   Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.RESTING), m_elevator));
    }  
      /****************** */
  }

  public void createAutos() {
    NamedCommands.registerCommand("Feed Coral", Commands.runOnce(m_coralMech::feed, m_coralMech));
    NamedCommands.registerCommand("Stop Feeding Coral", Commands.runOnce(m_coralMech::stop, m_coralMech));
    NamedCommands.registerCommand("Intake Algae", Commands.runOnce(m_algaeMech::intake, m_algaeMech));
    NamedCommands.registerCommand("Outtake Algae", Commands.runOnce(m_algaeMech::outtake, m_algaeMech));
    NamedCommands.registerCommand("Stop Algae Wheels", Commands.runOnce(m_algaeMech::stop, m_algaeMech));

    NamedCommands.registerCommand("L1", Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.L1), m_elevator));
    NamedCommands.registerCommand("L2", Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.L2), m_elevator));
    NamedCommands.registerCommand("L2 Offset", Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.L2_ALGAE_OFFSET), m_elevator));
    NamedCommands.registerCommand("L3", Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.L3), m_elevator));
    NamedCommands.registerCommand("L3 Offset", Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.L3_ALGAE_OFFSET), m_elevator));
    NamedCommands.registerCommand("L4", Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.L4), m_elevator));
    NamedCommands.registerCommand("L4 Offset", Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.L4_OFFSET), m_elevator));

    /**
     * [Color of starting zone][Location within starting zone][Field Area][Field Area Loaction][Number scored]
     */
    m_autoChooser.setDefaultOption("Timed Leave", new TimedLeave(m_drivebase));
    //m_autoChooser.addOption("Leave", new PathPlannerAuto("Leave"));
    //m_autoChooser.addOption("BlueTHexTRL4", new PathPlannerAuto("BlueTHexTRL4")); 
    m_autoChooser.addOption("BlueTHexTRAlgaeL3CoralL4", new PathPlannerAuto("BlueTHexTRAlgaeL3CoralL4")); 
    SmartDashboard.putData("Auto Choose", m_autoChooser);
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected(); // nothing selected should default to timed leave
  }
  
  public void setMotorBrake(boolean brake)
  {
    m_drivebase.setMotorBrake(brake);
  }
}
