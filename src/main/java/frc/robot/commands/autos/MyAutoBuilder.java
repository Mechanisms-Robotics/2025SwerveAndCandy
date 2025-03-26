package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.swervedrive.auto.CoralStationLineup;
import frc.robot.commands.swervedrive.auto.PIDtoPosition;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class MyAutoBuilder {
    public static Command twoCoral(SwerveSubsystem swerve) {
        var targetPose = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)
        ? FieldConstants.BLUE_REEF_POSES.get(20).getFirst()
        : FieldConstants.RED_REEF_POSES.get(9).getFirst();
        return new PIDtoPosition(swerve, targetPose).withTimeout(3.0);
    }
}
