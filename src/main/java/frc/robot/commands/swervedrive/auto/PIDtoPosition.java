package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * Go to a position using pid for x, y, and rotation compenents seperatly.
 * Very simple so easy to debug.
 */
public class PIDtoPosition extends Command {
    private final SwerveSubsystem swerve;
    private final Pose2d targetPosition;
    private final PIDController rotationController = new PIDController(0.05, 0, 0);
    private final PIDController xController = new PIDController(1.0, 0, 0);
    private final PIDController yController = new PIDController(1.0, 0, 0);
    private final double positionTolerance = 0.0;
    private final double rotationTolerance = 0.0;

    /**
     * PID to a given position. It pids x, y and rotational compents seperately.
     * Usedfull for its simplicity.
     * 
     * @param swerve used for driving
     * @param targetPosition position to be PIDed to
     */
    public PIDtoPosition(SwerveSubsystem swerve, Pose2d targetPosition) {
        this.swerve = swerve;
        this.targetPosition = targetPosition;

        // if this is not here, the rotational controller has nasty rap arounds errors
        this.rotationController.enableContinuousInput(-180.0, 180.0);
        
        // tollerance is used to exit the function when it is good enough
        this.rotationController.setTolerance(rotationTolerance);
        this.xController.setTolerance(positionTolerance);
        this.yController.setTolerance(positionTolerance);
        
        addRequirements(this.swerve);
    }


    @Override
    public void execute() {
        double velocityX = xController.calculate(swerve.getPose().getX(), targetPosition.getX()); // meters per second
        double velocityY = yController.calculate(swerve.getPose().getY(), targetPosition.getY()); // meters per second
        double radiansPerSecond = rotationController.calculate(swerve.getHeading().getDegrees(), targetPosition.getRotation().getDegrees());
        swerve.driveFieldOriented(new ChassisSpeeds(velocityX, velocityY, radiansPerSecond));
    }

    @Override
    public void end(boolean interupted) {
        // stop the swerve on end
        swerve.driveFieldOriented(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        // finish when the rotation target and position target are within tollerance
        return swerve.getPose().getTranslation().getDistance(targetPosition.getTranslation()) < positionTolerance
               && Math.abs(swerve.getHeading().getDegrees() - targetPosition.getRotation().getDegrees()) < rotationTolerance;
    }
}
