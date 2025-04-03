package frc.robot.commands.autos;

import java.lang.reflect.Field;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.ElevatorRest;
import frc.robot.commands.L2;
import frc.robot.commands.L3;
import frc.robot.commands.L4;
import frc.robot.commands.swervedrive.auto.BargeAlign;
import frc.robot.commands.swervedrive.auto.PIDtoPosition;
import frc.robot.subsystems.AlgaeMech;
import frc.robot.subsystems.CoralMech;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class MyAutoBuilder {
    public static Command twoCoralProcessorSide(SwerveSubsystem swerve, Elevator elevator, AlgaeMech algaeMech, CoralMech coralMech, DriverStation.Alliance alliance) {
        Pose2d reefTarget1 = alliance.equals(DriverStation.Alliance.Blue)
        ? FieldConstants.BLUE_REEF_POSES_AUTO.get(22).getSecond()
        : FieldConstants.RED_REEF_POSES_AUTO.get(9).getSecond();
        Pose2d reefTarget2 = alliance.equals(DriverStation.Alliance.Blue)
        ? FieldConstants.BLUE_REEF_POSES_AUTO.get(17).getSecond()
        : FieldConstants.RED_REEF_POSES_AUTO.get(8).getSecond();
        Pose2d coralStationTarget = alliance.equals(DriverStation.Alliance.Blue)
        ? FieldConstants.BLUE_CORAL_STATION_RIGHT
        : FieldConstants.RED_CORAL_STATION_RIGHT;

        String ntTable = "Commands/auto/2 L4 coral left/";

        return Commands.sequence(
            Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state", "I am driving to the reef (" + reefTarget1.toString() + ")"))
            , new PIDtoPosition(swerve, reefTarget1)
                .until(() -> swerve.isNear(reefTarget1))
                    .withTimeout(5)
            , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
                "I am raising the elevator to L4. I got " + swerve.getMyPose().getTranslation().getDistance(reefTarget1.getTranslation())
                + " meters close to my target"))
            , new L4(elevator)
                .until(elevator::atGoal)
                    .withTimeout(3)
            , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
              "I am feeding coral, waiting to stop"))
            , Commands.runOnce(coralMech::feed)
            , new WaitCommand(1)
            , Commands.runOnce(coralMech::stop)
            , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
              "I stopped feeding coral and am now lowering the elevator to rest"))
            , new ElevatorRest(elevator, algaeMech)
                .until(() -> Math.abs(elevator.getCurrentPosition() - Elevator.RESTING) < 5000)
                    .withTimeout(5)
            , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
              " I am now moving to the coral station"))
            , new PIDtoPosition(swerve, coralStationTarget)
                    .withTimeout(3.5)
            , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
              " I am now moving to my second reef target"))
            , new PIDtoPosition(swerve, reefTarget2)
                .until(() -> swerve.isNear(reefTarget2))
            , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
              "I am now moving elevator to L4"))
            , new L4(elevator)
                .until(elevator::atGoal)
                    .withTimeout(2)
            , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
              "I am now feeding coral"))
            , Commands.runOnce(coralMech::feed)
            , new WaitCommand(2)
            , Commands.runOnce(coralMech::stop)
            , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state", "I am lowering the elevator to rest"))
            , new ElevatorRest(elevator, algaeMech)
                .until(elevator::atGoal)
                    .withTimeout(5)
            , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state", "I am driving to the coral station again"))
            , new PIDtoPosition(swerve, coralStationTarget)
        );
    }

    public static Command twoCoralBargeSide(SwerveSubsystem swerve, Elevator elevator, AlgaeMech algaeMech, CoralMech coralMech, DriverStation.Alliance alliance) {
      Pose2d reefTarget1 = alliance.equals(DriverStation.Alliance.Blue)
      ? FieldConstants.BLUE_REEF_POSES_AUTO.get(20).getSecond()
      : FieldConstants.RED_REEF_POSES_AUTO.get(11).getSecond();
      Pose2d reefTarget2 = alliance.equals(DriverStation.Alliance.Blue)
      ? FieldConstants.BLUE_REEF_POSES_AUTO.get(19).getSecond()
      : FieldConstants.RED_REEF_POSES_AUTO.get(6).getSecond();
      Pose2d coralStationTarget = alliance.equals(DriverStation.Alliance.Blue)
      ? FieldConstants.BLUE_CORAL_STATION_LEFT
      : FieldConstants.RED_CORAL_STATION_LEFT;

      String ntTable = "Commands/auto/2 L4 coral right/";

      return Commands.sequence(
        Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state", "I am driving to the reef (" + reefTarget1.toString() + ")"))
        , new PIDtoPosition(swerve, reefTarget1)
            .until(() -> swerve.isNear(reefTarget1))
                .withTimeout(5)
        , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
            "I am raising the elevator to L4. I got " + swerve.getMyPose().getTranslation().getDistance(reefTarget1.getTranslation())
            + " meters close to my target"))
        , new L4(elevator)
            .until(elevator::atGoal)
                .withTimeout(3)
        , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
          "I am feeding coral, waiting to stop"))
        , Commands.runOnce(coralMech::feed)
        , new WaitCommand(1)
        , Commands.runOnce(coralMech::stop)
        , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
          "I stopped feeding coral and am now lowering the elevator to rest"))
        , new ElevatorRest(elevator, algaeMech)
            .until(() -> Math.abs(elevator.getCurrentPosition() - Elevator.RESTING) < 5000)
                .withTimeout(5)
        , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
          " I am now moving to the coral station"))
        , new PIDtoPosition(swerve, coralStationTarget)
                .withTimeout(3.5)
        , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
          " I am now moving to my second reef target"))
        , new PIDtoPosition(swerve, reefTarget2)
            .until(() -> swerve.isNear(reefTarget2))
        , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
          "I am now moving elevator to L4"))
        , new L4(elevator)
            .until(elevator::atGoal)
                .withTimeout(2)
        , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
          "I am now feeding coral"))
        , Commands.runOnce(coralMech::feed)
        , new WaitCommand(2)
        , Commands.runOnce(coralMech::stop)
        , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state", "I am lowering the elevator to rest"))
        , new ElevatorRest(elevator, algaeMech)
            .until(elevator::atGoal)
                .withTimeout(5)
        , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state", "I am driving to the coral station again"))
        , new PIDtoPosition(swerve, coralStationTarget)
      );
    }

    public static Command coralAndAlgaeCenterBarge(SwerveSubsystem swerve, Elevator elevator, AlgaeMech algaeMech, CoralMech coralMech, DriverStation.Alliance alliance) {
      Pose2d reefTarget1 = alliance.equals(DriverStation.Alliance.Blue)
      ? FieldConstants.BLUE_REEF_POSES_AUTO.get(21).getSecond()
      : FieldConstants.RED_REEF_POSES_AUTO.get(10).getSecond();
      double bargeTarget = alliance.equals(DriverStation.Alliance.Blue)
      ? FieldConstants.BLUE_BARG_POSE
      : FieldConstants.RED_BARG_POSE;
      Pose2d processorTarget = alliance.equals(DriverStation.Alliance.Blue)
      ? FieldConstants.BLUE_PROCESSOR_POSE
      : FieldConstants.RED_PROCESSOR_POSE;


      String ntTable = "Commands/auto/L4 coral and algae center/";

      return Commands.sequence(
          Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state", "I am driving to the reef (" + reefTarget1.toString() + ")"))
          , new PIDtoPosition(swerve, reefTarget1)
              .until(() -> swerve.isNear(reefTarget1))
                  .withTimeout(5)
          , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
              "I am raising the elevator to L4. I got " + swerve.getMyPose().getTranslation().getDistance(reefTarget1.getTranslation())
              + " meters close to my target"))
          , new L4(elevator)
              .until(elevator::atGoal)
                  .withTimeout(3)
          , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
            "I am feeding coral, waiting to stop"))
          , Commands.runOnce(coralMech::feed)
          , new WaitCommand(1)
          , Commands.runOnce(coralMech::stop)
          , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
            "I stopped feeding coral and am now lowering the elevator to get the algae"))
          , Commands.runOnce(() -> swerve.drive(new ChassisSpeeds(-0.75, 0.0, 0.0)))
            .withTimeout(0.5)
          , new L2(elevator, algaeMech, () -> true, () -> false, () -> false, () -> false) 
              .until(elevator::atGoal)
                  .withTimeout(2)
          , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
            " I am now intaking the algae"))
          , Commands.runOnce(algaeMech::intake)
          , Commands.run(() -> swerve.drive(new ChassisSpeeds(0.5, -0.05, 0.0)))
            .withTimeout(1)
          , Commands.run(() -> swerve.drive(new ChassisSpeeds(-0.5, 0.0, 0.0)))
            .withTimeout(1.5)
          , Commands.runOnce(() -> algaeMech.setWristAngle(AlgaeMech.WRIST_ANGLE_DOWN))
          , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
            " I am now moving the elevator to rest"))
          , Commands.runOnce(() -> elevator.setTargetPosition(Elevator.L1))
          , new WaitCommand(1)
          , new PIDtoPosition(swerve, processorTarget)
            .until(() -> swerve.isNear(processorTarget))
          , Commands.runOnce(algaeMech::outtake)
          // , Commands.runOnce(() -> swerve.drive(new ChassisSpeeds(0.0, 1.0, 0.0)))
          //   .withTimeout(2)
          
          // , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
          //   "I am now moving to barge"))
          // , new BargeAlign(swerve, () -> 0)
          //         .withTimeout(5)
          // , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
          //   "I am now raising the elevator to barge"))
          // , Commands.runOnce(algaeMech::outtake)
          // , new WaitCommand(1)
          // , Commands.runOnce(algaeMech::stop)
          // , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state", "I am lowering the elevator to rest"))
          // , new ElevatorRest(elevator, algaeMech)
          //     .until(elevator::atGoal)
          //         .withTimeout(5)
      );
  }

}

