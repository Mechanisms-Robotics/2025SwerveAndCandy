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
        addCommands(
            // Drives to the position using pathplanner, it is used first because it has path optimisation. Often, if it gets close the position
            // it "gives up". Since it is pathplanner, its functionality is obscured and it is hard to debug
            // This is currently comented out because pathplanner is using yagsls position which only (or at least should only) use odometry
            // if you want to uncomment this, configure the autobuilder to use my position localisation
            // new DeferredCommand(() -> swerve.driveToPose(findClosestTarget(swerve.getPose(), right.getAsBoolean())), getRequirements()),
            // After pathplanner "does its best" or "gives up" a simple pid controller to the position is used, note this rechecks if the right
            // boolean supplier is pressed. If leif holds right bumper when the pathplanner drive position is initalized, but not when the
            // pid controller drive position is initalized, it will start going to the left side of the reef when it finished path planner
            new DeferredCommand(() -> {
                var position = findClosestTarget(swerve.getMyPose(), right.getAsBoolean());
                return new PIDtoPosition(swerve, position);
            }, getRequirements())
        );
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



