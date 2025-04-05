// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ElevatorBarge;
import frc.robot.commands.ElevatorRest;
import frc.robot.commands.L2;
import frc.robot.commands.L3;
import frc.robot.commands.autos.MyAutoBuilder;
import frc.robot.commands.L4;
import frc.robot.commands.autos.TimedLeave;
import frc.robot.commands.swervedrive.auto.AutoReefLineup;
import frc.robot.commands.swervedrive.auto.BargeAlign;
import frc.robot.commands.swervedrive.auto.CoralStationLineup;
import frc.robot.subsystems.AlgaeMech;
import frc.robot.subsystems.CoralMech;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLight;
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

  // toggle setting for CONFIRM MODE for elevator commands - defaults to TRUE
  private boolean m_shiftConfirmMode = true;

  // Replace with CommandPS4Controller or CommandJoystick if needed

  final         CommandPS4Controller driverController = new CommandPS4Controller(0);

  private final CommandXboxController secondaryController = new CommandXboxController(3);

  private final CommandJoystick pedals = new CommandJoystick(1);

  private final Joystick shifter = new Joystick(2);

  
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
    "swerve"));
  public final Elevator m_elevator = new Elevator();
  public final CoralMech m_coralMech = new CoralMech();
  public final AlgaeMech m_algaeMech = new AlgaeMech();
  public final LimeLight m_limeLight1 = new LimeLight(Constants.LimeLight1.NICKNAME, Constants.LimeLight1.FIELD_TO_CAMERA, m_drivebase);
  //public final LimeLight m_limeLight2 = new LimeLight(Constants.LimeLight2.NICKNAME, Constants.LimeLight2.FIELD_TO_CAMERA, m_drivebase);
  
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  private Supplier<Boolean> m_algaeClutch = () -> false;
  private Supplier<Boolean> m_coralClutch = () -> false;

  private Supplier<Boolean> m_elevUp = () -> false;
  private Supplier<Boolean> m_elevDown = () -> false;

  private static final double LEFT_PEDAL_THRESHOLD = 0.2;
  private static final double RIGHT_PEDAL_THRESHOLD = -0.8;
  private static final double MIDDLE_PEDAL_THRESHOLD = -0.6;

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
      // CommandPS4Controller rotationJoystick = new CommandPS4Controller(1);
      // SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
      //   () -> Math.pow(driverController.getLeftY(), 3),
      //   () -> Math.pow(driverController.getLeftX(), 3))
      //     .withControllerRotationAxis(() -> -Math.signum(rotationJoystick.getRawAxis(0))*Math.pow(rotationJoystick.getRawAxis(0), 2))
      //     .deadband(OperatorConstants.DEADBAND)
      //     .scaleTranslation(0.8)
      //     .allianceRelativeControl(true);
      // Command driveFieldOrientedAnglularVelocityKeyboard = m_drivebase.driveFieldOriented(driveAngularVelocityKeyboard);

      CommandXboxController xboxController = new CommandXboxController(0);
      SwerveInputStream driveAngularVelocityXbox = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
        () -> Math.pow(xboxController.getLeftY(), 3),
        () -> Math.pow(xboxController.getLeftX(), 3))
          .withControllerRotationAxis(() -> -Math.signum(xboxController.getRightX())*Math.pow(xboxController.getRightX(), 2))
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);
      Command driveFieldOrientedAnglularVelocityXbox = m_drivebase.driveFieldOriented(driveAngularVelocityXbox);

      xboxController.a().whileTrue(new AutoReefLineup(m_drivebase, xboxController.rightBumper()));
      xboxController.b().whileTrue(new BargeAlign(m_drivebase, () -> -xboxController.getLeftX()));
      xboxController.x().whileTrue(new CoralStationLineup(m_drivebase));
      xboxController.start().onTrue(Commands.runOnce(m_drivebase::zeroGyro));
      m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityXbox);
    } else
    {
      m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      // Testing Brennan's controller
      DriverStation.reportWarning("Simulation Mode!",false);

      configureSimSecondaryControllers();

      // driveDirectAngleKeyboard.driveToPose(() -> new Pose2d(new Translation2d(9, 3),
      //                                                       Rotation2d.fromDegrees(90)),
      //                                      new ProfiledPIDController(5,
      //                                                                0,
      //                                                                0,
      //                                                                new Constraints(5,
      //                                                                                3)),
      //                                      new ProfiledPIDController(5,
      //                                                                0,
      //                                                                0,
      //                                                                new Constraints(
      //                                                                    Math.toRadians(
      //                                                                        360),
      //                                                                    Math.toRadians(
      //                                                                        90))));
      // driverController.options().onTrue(Commands.runOnce(() -> m_drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      // driverController.button(1).whileTrue(m_drivebase.sysIdDriveMotorCommand());
      // driverController.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
      //                                                () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

      // // Joystick buttons = new Joystick(0);
      // // Supplier<Boolean> clutch = () -> buttons.getRawButton(4);
      // // new Trigger(() -> buttons.getRawButtonPressed(1)).onTrue(new L2(m_elevator, m_algaeMech, clutch, secondaryController));
      // // new Trigger(() -> buttons.getRawButtonPressed(2)).onTrue(new L3(m_elevator, m_algaeMech, clutch));
    }
    if (DriverStation.isTest())
    {
      // m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      // driverController.square().whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly());
      // driverController.triangle().whileTrue(m_drivebase.driveToDistanceCommand(1.0, 0.2));
      // driverController.options().onTrue((Commands.runOnce(m_drivebase::zeroGyro)));
      // driverController.share().whileTrue(m_drivebase.centerModulesCommand());
      // driverController.L2().onTrue(Commands.none());
      // driverController.R2().onTrue(Commands.none());
    } else
    /**
     * 
     ******** The following code is the acutal drive code used *******
     * 
     * 
     */
    {
      driverController.share().onTrue(Commands.runOnce(m_drivebase::zeroGyro));
      // setup secondaryController and pedals for REAL physical mode
      configureRealSecondaryControllers();


      // driverController.touchpad().whileTrue(new PointAtApriltag(m_drivebase, m_limeLight, 5));

      // driverController.touchpad().whileTrue(new DriveWhileApriltagPoint(m_drivebase, m_limeLight1,
      //   () -> driverController.getLeftX(),
      //   () -> driverController.getLeftY(),
      //   () -> driverController.getRightX(), Constants.FieldConstants.REEF_APRIL_TAGS));

      driverController.L1().whileTrue(new AutoReefLineup(m_drivebase, ()->false));
      driverController.R1().whileTrue(new AutoReefLineup(m_drivebase, ()->true));
        
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
      driverController.cross().onTrue(Commands.runOnce(m_coralMech::feed, m_coralMech));
      driverController.cross().onFalse(Commands.runOnce(m_coralMech::stop, m_coralMech));
      driverController.touchpad().onTrue(Commands.runOnce(m_coralMech::retract, m_coralMech));
      driverController.touchpad().onFalse(Commands.runOnce(m_coralMech::stop, m_coralMech));

      // Elevator
      // Move elevator up and down manualy, kept here for now. I have no particular commitment to keeping these here
      // May be removed if at all needed. - Micah
      driverController.povDown().onTrue(Commands.runOnce(
        () -> m_elevator.setTargetPosition(m_elevator.getCurrentPosition() - 500), m_elevator)
      );
        
      driverController.povUp().onTrue(Commands.runOnce(
        () -> m_elevator.setTargetPosition(m_elevator.getCurrentPosition() + 2000), m_elevator)
      );
    }  
  }

  public void createAutos() {
    NamedCommands.registerCommand("Feed Coral", Commands.runOnce(m_coralMech::feed, m_coralMech));
    NamedCommands.registerCommand("Stop Feeding Coral", Commands.runOnce(m_coralMech::stop, m_coralMech));
    NamedCommands.registerCommand("Intake Algae", Commands.runOnce(m_algaeMech::intake, m_algaeMech));
    NamedCommands.registerCommand("Outtake Algae", Commands.runOnce(m_algaeMech::outtake, m_algaeMech));
    NamedCommands.registerCommand("Stop Algae Wheels", Commands.runOnce(m_algaeMech::stop, m_algaeMech));

    NamedCommands.registerCommand("RestElevator", Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.RESTING), m_elevator));
    NamedCommands.registerCommand("L2", Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.L2), m_elevator));
    NamedCommands.registerCommand("L2 Offset", Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.L2_ALGAE_OFFSET), m_elevator));
    NamedCommands.registerCommand("L3", Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.L3), m_elevator));
    NamedCommands.registerCommand("L3 Offset", Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.L3_ALGAE_OFFSET), m_elevator));
    NamedCommands.registerCommand("L4", Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.L4), m_elevator));
    NamedCommands.registerCommand("L4 Offset", Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.L4_Offset), m_elevator));

    /**
     * [Color of starting zone][Location within starting zone][Field Area][Field Area Loaction][Number scored]
     */
    m_autoChooser.setDefaultOption("Timed Leave", new TimedLeave(m_drivebase));
    //m_autoChooser.addOption("Leave", new PathPlannerAuto("Leave"));
    //m_autoChooser.addOption("BlueTHexTRL4", new PathPlannerAuto("BlueTHexTRL4")); 

    // m_autoChooser.addOption("L4 Coral", new PathPlannerAuto("L4 Coral")); 
    // m_autoChooser.addOption("L2 Coral", new PathPlannerAuto("L2 Coral")); 
    // m_autoChooser.addOption("L2 Coral Reflected", new PathPlannerAuto("L2 Coral Reflected")); 
    // m_autoChooser.addOption("Test Auto", new PathPlannerAuto("Test Auto"));

    m_autoChooser.addOption("Red Two Coral Barge Side", MyAutoBuilder.twoCoralBargeSide(m_drivebase, m_elevator, m_algaeMech, m_coralMech, DriverStation.Alliance.Red));
    m_autoChooser.addOption("Red Two Coral Proccessor Side", MyAutoBuilder.twoCoralProcessorSide(m_drivebase, m_elevator, m_algaeMech, m_coralMech, DriverStation.Alliance.Red));
    m_autoChooser.addOption("Red Coral and Algae Center", MyAutoBuilder.coralAndAlgaeCenterBarge(m_drivebase, m_elevator, m_algaeMech, m_coralMech, DriverStation.Alliance.Red));
    m_autoChooser.addOption("Red Three Coral Barge Side", MyAutoBuilder.threeCoralProcessorSide(m_drivebase, m_elevator, m_algaeMech, m_coralMech, DriverStation.Alliance.Red));
    
    m_autoChooser.addOption("Blue Two Coral Barge Side", MyAutoBuilder.twoCoralBargeSide(m_drivebase, m_elevator, m_algaeMech, m_coralMech, DriverStation.Alliance.Blue));
    m_autoChooser.addOption("Blue Two Coral Proccessor Side", MyAutoBuilder.twoCoralProcessorSide(m_drivebase, m_elevator, m_algaeMech, m_coralMech, DriverStation.Alliance.Blue));
    m_autoChooser.addOption("Blue Coral and Algae Center", MyAutoBuilder.coralAndAlgaeCenterBarge(m_drivebase, m_elevator, m_algaeMech, m_coralMech, DriverStation.Alliance.Blue));
    m_autoChooser.addOption("Blue Three Coral Barge Side", MyAutoBuilder.threeCoralProcessorSide(m_drivebase, m_elevator, m_algaeMech, m_coralMech, DriverStation.Alliance.Blue));

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

  private void configureSimSecondaryControllers() {

    Trigger trigger_button1 = secondaryController.button(1);
    Trigger trigger_button2 = secondaryController.button(2);
    Trigger trigger_button3 = secondaryController.button(3);
    Trigger trigger_button4 = secondaryController.button(4);
    Trigger trigger_button5 = secondaryController.button(5);
    Trigger trigger_button6 = secondaryController.button(6);
    Trigger trigger_button7 = secondaryController.button(7);
    Trigger trigger_button8 = secondaryController.button(8);

    Trigger trigger_leftTrigger = secondaryController.leftTrigger(0.3);
    Trigger trigger_rightTrigger = secondaryController.rightTrigger(0.3);
    Trigger trigger_povLeft = secondaryController.povLeft();
    Trigger trigger_povRight = secondaryController.povRight();
    Trigger trigger_povUp = secondaryController.povUp();
    Trigger trigger_povDown = secondaryController.povDown();

    Trigger trigger_leftPedal = pedals.axisGreaterThan(0, LEFT_PEDAL_THRESHOLD);
    Trigger trigger_middlePedal = pedals.axisGreaterThan(2, MIDDLE_PEDAL_THRESHOLD);
    Trigger trigger_rightPedal = pedals.axisGreaterThan(3, RIGHT_PEDAL_THRESHOLD);

    trigger_leftPedal.onTrue(new PrintCommand("Left Pedal pressed"));
    trigger_middlePedal.onTrue(new PrintCommand("Middle Pedal pressed"));
    trigger_rightPedal.onTrue(new PrintCommand("Right Pedal pressed"));

    trigger_button1.onTrue(new PrintCommand("Button1 pressed"));
    trigger_button2.onTrue(new PrintCommand("Button2 pressed"));
    trigger_button3.onTrue(new PrintCommand("Button3 pressed"));
    trigger_button4.onTrue(new PrintCommand("Button4 pressed"));
    trigger_button5.onTrue(new PrintCommand("Button5 pressed"));
    trigger_button6.onTrue(new PrintCommand("Button6 pressed"));
    trigger_button7.onTrue(new PrintCommand("Button7 pressed"));
    trigger_button8.onTrue(new PrintCommand("Button8 pressed"));

    trigger_leftTrigger.onTrue(new PrintCommand("Left Trigger pressed"));
    trigger_rightTrigger.onTrue(new PrintCommand("Right Trigger pressed"));

    trigger_povLeft.onTrue(new PrintCommand("pov Left pressed"));
    trigger_povRight.onTrue(new PrintCommand("pov Right pressed"));
    trigger_povUp.onTrue(new PrintCommand("pov Up pressed"));
    trigger_povDown.onTrue(new PrintCommand("pov Down pressed"));
  }

  private void configureRealSecondaryControllers()
  {
    // shifter triggers
    Trigger trigger_shifter_1 = new Trigger(() -> shifter.getRawButtonPressed(1));

    // since L2 - L4 leverage offsets and clutch operation, they need continuous state of shifter,
    // thus using getRawButton()
    Trigger trigger_shifter_2 = new Trigger(() -> shifter.getRawButton(2));
    Trigger trigger_shifter_3 = new Trigger(() -> shifter.getRawButton(3));
    Trigger trigger_shifter_4 = new Trigger(() -> shifter.getRawButton(4));

    Trigger trigger_shifter_5 = new Trigger(() -> shifter.getRawButton(5));
    Trigger trigger_shifter_6 = new Trigger(() -> shifter.getRawButton(6));
    Trigger trigger_shifter_7 = new Trigger(() -> shifter.getRawButton(7));
    Trigger trigger_shifter_8 = new Trigger(() -> shifter.getRawButton(8));
    
    // using Brennan's Saber FGC controller
    Trigger trigger_barge = secondaryController.povRight();
    Trigger trigger_rest = secondaryController.povUp();
    Trigger trigger_L1 = secondaryController.button(1);
    Trigger trigger_L2 = secondaryController.button(2);
    Trigger trigger_L3 = secondaryController.leftTrigger();
    Trigger trigger_L4 = secondaryController.rightTrigger();
    Trigger trigger_elevDown = secondaryController.button(3);
    Trigger trigger_elevUp = secondaryController.button(4);
    Trigger trigger_wristDown = secondaryController.button(5);
    Trigger trigger_wristUp = secondaryController.button(6);
    Trigger trigger_algaeClutch = secondaryController.povLeft();
    Trigger trigger_coralClutch = secondaryController.povDown();

    Trigger trigger_eRestPedal = pedals.axisGreaterThan(0, LEFT_PEDAL_THRESHOLD);
    Trigger trigger_algaeClutchPedal = pedals.axisGreaterThan(2, MIDDLE_PEDAL_THRESHOLD);
    Trigger trigger_coralClutchPedal = pedals.axisGreaterThan(3, RIGHT_PEDAL_THRESHOLD);

    // combine redundant controls into single trigger for simpler logic below
    Trigger trigger_coralClutchDuo = trigger_coralClutch.or(trigger_coralClutchPedal);
    Trigger trigger_algaeClutchDuo = trigger_algaeClutch.or(trigger_algaeClutchPedal);
    Trigger trigger_bargeDuo = trigger_barge.or(trigger_shifter_7);
    Trigger trigger_restDuo = trigger_rest.or(trigger_shifter_8);
    Trigger trigger_L1Duo = trigger_L1.or(trigger_shifter_1);
    Trigger trigger_L2Duo = trigger_L2.or(trigger_shifter_2);
    Trigger trigger_L3Duo = trigger_L3.or(trigger_shifter_3);
    Trigger trigger_L4Duo = trigger_L4.or(trigger_shifter_4);

    // left pedal is emergency rest
    trigger_eRestPedal.onTrue(new ElevatorRest(m_elevator, m_algaeMech, ()->false));

    //trigger_eRestPedal.onTrue(Commands.runOnce(stateMachine.setState(StateMachine.State.DrivingToReefTarget), stateMachine));

    // Set BooleanSuppliers based on clutch and elevUp/elevDown states
    m_algaeClutch = () -> trigger_algaeClutch.or(trigger_algaeClutchPedal).getAsBoolean();
    m_coralClutch = () -> trigger_coralClutch.or(trigger_coralClutchPedal).getAsBoolean();
    Supplier<Boolean> L4_clutch = () -> m_algaeClutch.get() || m_coralClutch.get();
    m_elevUp = () -> trigger_elevUp.getAsBoolean();
    m_elevDown = () -> trigger_elevDown.getAsBoolean();

    // Toggle CONFIRM MODE if cheat code enabled
    //   cheat code = Barge + Rest + CoralClutch + AlgaeClutch all on gamePad
    // 
    // If team likes original mode better, the default at top of the file should be flipped
    // so they don't have to toggle on gamepad for each match.
    trigger_barge.and(trigger_rest).and(trigger_coralClutch).and(trigger_algaeClutch).onTrue(
      Commands.runOnce(() -> toggleConfirmMode())
    );

    if (m_shiftConfirmMode) {
    /**** BAD CODING PRACTICE BELOW [M.Fox] ***
      We could end up in a situation where the shifter is left in one position, say L3, and the
      gamepad requests a different position by pressing, say L2. Now the OR conditions below should
      theoretically trigger both L2 and L3 commands to be called by the scheduler which could result
      in indeterminate behavior. Through testing it appears that the scheduler handles this by simply
      letting the last-executed command win. This is logical, and is fine for competition, but still
      bad coding nonetheless. We should try to avoid such scenarios.
    */

      // Only trigger when coral clutch (CONFIRM) AND L(n) is pressed for all of these commands
      trigger_coralClutchDuo.and(trigger_L1Duo).onTrue(Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.L1), m_elevator));
      // Since L2-L4 have elevBumps, they need to be triggered continuously to update 
      // L2 when coral clutch (CONFIRM) AND L2 is pressed
      trigger_coralClutchDuo.and(trigger_L2Duo).whileTrue(new L2(m_elevator, m_algaeMech, ()->false, m_coralClutch, m_elevUp, m_elevDown));
      trigger_coralClutchDuo.and(trigger_L3Duo).whileTrue(new L3(m_elevator, m_algaeMech, ()->false, m_coralClutch, m_elevUp, m_elevDown));
      trigger_coralClutchDuo.and(trigger_L4Duo).whileTrue(new L4(m_elevator, L4_clutch, m_elevUp, m_elevDown));

      trigger_coralClutchDuo.and(trigger_shifter_6).onTrue(Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.PROCESSOR), m_elevator));
      trigger_coralClutchDuo.and(trigger_bargeDuo).onTrue(new ElevatorBarge(m_elevator, m_algaeMech));
      trigger_coralClutchDuo.and(trigger_restDuo).onTrue(new ElevatorRest(m_elevator, m_algaeMech, ()->false));
      
      trigger_wristDown.whileTrue(Commands.run(() -> m_algaeMech.bumpWristUp(-AlgaeMech.WRIST_BUMP)));
      trigger_wristUp.whileTrue(Commands.run(() -> m_algaeMech.bumpWristUp(AlgaeMech.WRIST_BUMP)));

      // ALGAE MODE TRIGGERS FOR L2 and L3 ONLY WHEN ALGAE_CONFIRM IS PRESSED
      trigger_algaeClutchDuo.and(trigger_L1Duo).onTrue(Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.L1), m_elevator));      
      trigger_algaeClutchDuo.and(trigger_L2Duo).whileTrue(new L2(m_elevator, m_algaeMech, m_algaeClutch, ()->false, m_elevUp, m_elevDown));
      trigger_algaeClutchDuo.and(trigger_L3Duo).whileTrue(new L3(m_elevator, m_algaeMech, m_algaeClutch, ()->false, m_elevUp, m_elevDown));
      // Ada said it would be intutive for the barge to be confirmed on algae clutch because barge does algae stuff
      trigger_algaeClutchDuo.and(trigger_bargeDuo).onTrue(new ElevatorBarge(m_elevator, m_algaeMech));
      trigger_algaeClutchDuo.and(trigger_restDuo).whileTrue(new ElevatorRest(m_elevator, m_algaeMech, ()->false));
      trigger_algaeClutchDuo.and(trigger_L4Duo).whileTrue(new L4(m_elevator, L4_clutch, m_elevUp, m_elevDown));

    } else{ // original mode, SHIFT_CONFIRM_MODE = false

    /**** BAD CODING PRACTICE BELOW [M.Fox] ***
      We could end up in a situation where the shifter is left in one position, say L3, and the
      gamepad requests a different position by pressing, say L2. Now the OR conditions below should
      theoretically trigger both L2 and L3 commands to be called by the scheduler which could result
      in indeterminate behavior. Through testing it appears that the scheduler handles this by simply
      letting the last-executed command win. This is logical, and is fine for competition, but still
      bad coding nonetheless. We should try to avoid such scenarios.
    */

      trigger_L1.or(trigger_shifter_1).onTrue(Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.L1), m_elevator));

      // Since these have two different modes, they need to be triggered continnously to update the mode in the event the clutch is engaged
      trigger_L2.or(trigger_shifter_2).whileTrue(new L2(m_elevator, m_algaeMech, m_algaeClutch, m_coralClutch, m_elevUp, m_elevDown));
      trigger_L3.or(trigger_shifter_3).whileTrue(new L3(m_elevator, m_algaeMech, m_algaeClutch, m_coralClutch, m_elevUp, m_elevDown));
      trigger_L4.or(trigger_shifter_4).whileTrue(new L4(m_elevator, L4_clutch, m_elevUp, m_elevDown));

      trigger_shifter_6.onTrue(Commands.runOnce(() -> m_elevator.setTargetPosition(Elevator.PROCESSOR), m_elevator));
      trigger_barge.or(trigger_shifter_7).onTrue(new ElevatorBarge(m_elevator, m_algaeMech));
      trigger_rest.or(trigger_shifter_8).onTrue(new ElevatorRest(m_elevator, m_algaeMech, m_algaeClutch));
      
      trigger_wristDown.whileTrue(Commands.run(() -> m_algaeMech.bumpWristUp(-AlgaeMech.WRIST_BUMP)));
      trigger_wristUp.whileTrue(Commands.run(() -> m_algaeMech.bumpWristUp(AlgaeMech.WRIST_BUMP)));
    }
  }

  private void toggleConfirmMode() {
    m_shiftConfirmMode = !m_shiftConfirmMode;
    DriverStation.reportWarning("CONFIRM_MODE CHANGED! confirmMode = "+String.valueOf(m_shiftConfirmMode),false);
  }
}
