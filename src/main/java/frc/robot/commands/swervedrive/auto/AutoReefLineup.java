package frc.robot.commands.swervedrive.auto;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoReefLineup extends SequentialCommandGroup {

    private final StructPublisher<Pose2d> targetPositionPublisher;

    public AutoReefLineup(SwerveSubsystem swerve, BooleanSupplier right) {
        addRequirements(swerve);
        addCommands(new DeferredCommand(() -> swerve.driveToPose(findClosestTarget(swerve.getPose(), right.getAsBoolean())), getRequirements()));
        targetPositionPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard")
            .getStructTopic("Target Position Pose2d", Pose2d.struct).publish();

    }
    
    public  Pose2d findClosestTarget(Pose2d current, boolean right) {

        if (current == null) {
            return null;
        }
        int[] ids = new int[Constants.FieldConstants.BLUE_REEF_APRIL_TAGS.length + Constants.FieldConstants.RED_REEF_APRIL_TAGS.length];
        
        System.arraycopy(Constants.FieldConstants.BLUE_REEF_APRIL_TAGS, 0, ids, 0, Constants.FieldConstants.BLUE_REEF_APRIL_TAGS.length);
        System.arraycopy(Constants.FieldConstants.RED_REEF_APRIL_TAGS, 0, ids, Constants.FieldConstants.BLUE_REEF_APRIL_TAGS.length, Constants.FieldConstants.RED_REEF_APRIL_TAGS.length);
        
        HashMap<Integer, Pair<Pose2d, Pose2d>> targets = new HashMap<Integer, Pair<Pose2d, Pose2d>>();
        targets.putAll(Constants.FieldConstants.RED_REEF_POSES);
        targets.putAll(Constants.FieldConstants.BLUE_REEF_POSES);

        Pose2d closest = null;
        double minDistance = Double.MAX_VALUE;

        for (int id : ids) {
            Pose2d target = (right) ? targets.get(id).getSecond() : targets.get(id).getFirst();
            double distance = current.getTranslation().getDistance(target.getTranslation());
            if (distance <= minDistance) {
                minDistance = distance;
                closest = target;
            }
        }

        targetPositionPublisher.set(closest);

        return closest;
    }
    
}



