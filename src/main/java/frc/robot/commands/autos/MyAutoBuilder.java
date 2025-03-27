package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.ElevatorRest;
import frc.robot.commands.L4;
import frc.robot.commands.swervedrive.auto.DriveCommands;
import frc.robot.subsystems.AlgaeMech;
import frc.robot.subsystems.CoralMech;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class MyAutoBuilder {
    public static Command twoCoral(SwerveSubsystem swerve, Elevator elevator, AlgaeMech algaeMech, CoralMech coralMech) {
        Pose2d reefTarget1 = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)
        ? FieldConstants.BLUE_REEF_POSES.get(20).getSecond()
        : FieldConstants.RED_REEF_POSES.get(9).getSecond();
        Pose2d reefTarget2 = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)
        ? FieldConstants.BLUE_REEF_POSES.get(19).getSecond()
        : FieldConstants.RED_REEF_POSES.get(8).getSecond();
        Pose2d coralStationTarget = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)
        ? FieldConstants.BLUE_CORAL_STATION_RIGHT
        : FieldConstants.RED_CORAL_STATION_RIGHT;

        String ntTable = "Commands/auto/2 L4 coral/";

        return Commands.sequence(
            Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state", "I am driving to the reef (" + reefTarget1.toString() + ")"))
            , new DriveCommands.PID(swerve, reefTarget1)
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
            , new DriveCommands.PID(swerve, coralStationTarget)
                .until(() -> swerve.isNear(coralStationTarget))
                    .withTimeout(5)
            , new WaitCommand(2)
            , Commands.runOnce(() -> SmartDashboard.putString(ntTable + "state",
              " I am now moving to my second reef target"))
            , new DriveCommands.PID(swerve, reefTarget2)
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
            , new DriveCommands.PID(swerve, coralStationTarget)
        );
    }

    public static Command twoCoralPathplanner(SwerveSubsystem swerve, Elevator elevator, AlgaeMech algaeMech, CoralMech coralMech) {
        Pose2d firstReef = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)
        ? FieldConstants.BLUE_REEF_POSES.get(20).getSecond()
        : FieldConstants.RED_REEF_POSES.get(9).getSecond();
        return Commands.sequence(
          DriveCommands.pathToPose(swerve, firstReef)
        );
    }
}
