package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoReefLineup extends SequentialCommandGroup {

    public AutoReefLineup(SwerveSubsystem swerve) {
        addRequirements(swerve);
        addCommands(swerve.driveToPose(findClosestTarget(swerve.getPose(), (
            DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue))
            ? Constants.FieldConstants.BLUE_REEF_POSES : Constants.FieldConstants.RED_REEF_POSES)));
    }
    
    public static Pose2d findClosestTarget(Pose2d current, Pose2d[] targets) {
        if (current == null) {
            return null;
        }
        if (targets == null) {
            throw new IllegalArgumentException("Target list cannot be null or empty.");
        }

        Pose2d closest = null;
        double minDistance = Double.MAX_VALUE;

        for (Pose2d target : targets) {
            double distance = current.getTranslation().getDistance(target.getTranslation());
            if (distance <= minDistance) {
                minDistance = distance;
                closest = target;
            }
        }

        return closest;
    }
    
}
