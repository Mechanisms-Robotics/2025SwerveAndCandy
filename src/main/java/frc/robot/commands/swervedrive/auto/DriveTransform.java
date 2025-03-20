package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveTransform extends Command {
    private final SwerveSubsystem swerve;
    private final Pose2d targetPosition;
    private final PIDController rotationController = new PIDController(0.05, 0, 0);
    private final PIDController xController = new PIDController(1.0, 0, 0);
    private final PIDController yController = new PIDController(1.0, 0, 0);

    public DriveTransform(SwerveSubsystem swerve, Transform2d transform) {
        this.swerve = swerve;
        this.targetPosition = swerve.getPose().transformBy(transform);
        xController.setSetpoint(targetPosition.getX());
        yController.setSetpoint(targetPosition.getY());
        rotationController.setSetpoint(targetPosition.getRotation().getDegrees());
        rotationController.enableContinuousInput(-180.0, 180.0);
        addRequirements(this.swerve);
    }

    @Override
    public void execute() {
        Pose2d pose = swerve.getPose();
        double xOutput = xController.calculate(pose.getX());
        double yOutput = xController.calculate(pose.getY());
        double rotOutput = rotationController.calculate(pose.getRotation().getDegrees());
        ChassisSpeeds speeds = new ChassisSpeeds(xOutput, yOutput, rotOutput);

        swerve.drive(speeds);
    }
    
}
