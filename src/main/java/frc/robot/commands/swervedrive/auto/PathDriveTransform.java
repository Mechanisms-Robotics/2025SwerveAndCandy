package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * Drives robot relative to a transform, or a position that is offset by the current position of the robot by the given transform.
 */
public class PathDriveTransform extends SequentialCommandGroup {
    private final SwerveSubsystem swerve;
    /**
     * Uses pathplanner to drive to a position that is the robots current read position (which is inteded to be only odometry)
     * offset by the given transform2d.
     * Note, this is super buggy because if the robot thinks it is next to the reef (or even inside of it because it is odometry)
     * it will fight you, likely not letting you go through what it thinks the reef is or whatever solid object is objects to.
     * 
     * @param swerve 
     * @param transform the offset of the current position that the robot will drive to 
     */
    public PathDriveTransform(SwerveSubsystem swerve, Transform2d transform) {
        this.swerve = swerve;
        addRequirements(this.swerve);
        addCommands(swerve.driveToPose(swerve.getPose().transformBy(transform)));
    }
}