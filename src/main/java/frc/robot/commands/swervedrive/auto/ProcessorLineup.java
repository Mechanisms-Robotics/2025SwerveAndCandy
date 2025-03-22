package frc.robot.commands.swervedrive.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ProcessorLineup extends SequentialCommandGroup {
    private final SwerveSubsystem swerve;
    public ProcessorLineup(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(this.swerve);
        addCommands(new DeferredCommand(() -> new PIDtoPosition(this.swerve, findClosestTarget()), getRequirements()));
    }

    public Pose2d findClosestTarget() {
        List<Pose2d> Poses = List.of(FieldConstants.BLUE_PROCESSOR_LEFT, FieldConstants.BLUE_PROCESSOR_RIGHT, FieldConstants.RED_PROCESSOR_LEFT, FieldConstants.RED_PROCESSOR_RIGHT);
        
        Pose2d closest = null;
        double closestDistance = Double.MAX_VALUE;
        for (Pose2d pose : Poses) {
            double distance = pose.getTranslation().getDistance(swerve.getMyPose().getTranslation());
            if (distance < closestDistance) {
                closestDistance = distance;
                closest = pose;
            }
        }
        if (closest == null)
            return swerve.getMyPose();
        return closest;
    }
}
