package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.LimeLight.ApriltagData;
import frc.robot.subsystems.StateMachine.State;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * Go to a position using pid for x, y, and rotation compenents seperatly.
 * Very simple so easy to debug.
 */
public class PIDtoPosition extends Command {
    private final SwerveSubsystem swerve;
    private final Pose2d targetPosition;
    private final PIDController rotationController = new PIDController(0.07, 0, 0);
    private final PIDController driveController = new PIDController(3.0, 0, 0);
    private final double positionTolerance = 0.0;
    private final double rotationTolerance = 0.0;
    private final double maxVelocity = 2.0;
    private final double maxRotationVelocity = Math.PI;

    // outputs the direction the robot is trying to go, this is meant for visualisation
    private final StructPublisher<Pose2d> pidOutputPublisher;
    private final StructPublisher<Pose2d> targetPositionPublisher;

    /**
     * PID to a given position. It pids x, y and rotational compents seperately.
     * Usedfull for its simplicity.
     * This is using my experimental position localisation
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

        String ntFolder = "SmartDashboard/Commands/PIDtoPosition/";
        pidOutputPublisher = NetworkTableInstance.getDefault()
          .getStructTopic(ntFolder + "output vector", Pose2d.struct).publish();
        targetPositionPublisher = NetworkTableInstance.getDefault()
          .getStructTopic(ntFolder + "target position", Pose2d.struct).publish();
        SmartDashboard.putData("Commands/PIDtoPosition/translation pidcontroller", driveController);
        SmartDashboard.putData("Commands/PIDtoPosition/rotation pidcontroller", rotationController);
        
        addRequirements(this.swerve);
    }

    @Override
    public void execute() {
        Pose2d currentPosition = swerve.getMyPose();
        // the magnitude (meaning it will always be positive) of the displacement between the two positions
        double currentDistance = currentPosition.getTranslation().getDistance(targetPosition.getTranslation());
        // the angle between the two positions, i.e. the angle the robot needs to drive at to arrive at the target position
        Rotation2d angleToTarget = new Rotation2d(
            Math.atan2(targetPosition.getY() - currentPosition.getY(), targetPosition.getX() - currentPosition.getX())
        );
        // the velocity calcuated by the distance as the error to the controller
        double velocity = -driveController.calculate(currentDistance, 0.0);
        velocity = MathUtil.clamp(velocity, -maxVelocity, maxVelocity);
        double velocityX = velocity * angleToTarget.getCos();
        double velocityY = velocity * angleToTarget.getSin();

        double radiansPerSecond = rotationController.calculate(currentPosition.getRotation().getDegrees(), targetPosition.getRotation().getDegrees());
        radiansPerSecond = MathUtil.clamp(radiansPerSecond, -maxRotationVelocity, maxRotationVelocity);

        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(velocityX, velocityY, radiansPerSecond);
        swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, currentPosition.getRotation()));

        Rotation2d vectorAngle = Rotation2d.fromRadians(Math.atan2(velocityY, velocityX));
        pidOutputPublisher.set(new Pose2d(swerve.getMyPose().getX(), currentPosition.getY(), vectorAngle));
        targetPositionPublisher.set(targetPosition);

        final double ARRIVED_EPSILON_METERS = 0.05;
        StateMachine.setState(
            currentDistance < ARRIVED_EPSILON_METERS ? State.ArrivedAtReefTarget : State.DrivingToReefTarget
        );
    }

    @Override
    public void end(boolean interupted) {
        // stop the swerve on end
        swerve.driveFieldOriented(new ChassisSpeeds());
        if (ApriltagData.hasTarget)
            StateMachine.setState(State.DetectedReefTarget);
        else
            StateMachine.setState(State.NoTargetIdentified);
    }

    @Override
    public boolean isFinished() {
        // finish when the rotation target and position target are within tollerance
        return swerve.getMyPose().getTranslation().getDistance(targetPosition.getTranslation()) < positionTolerance
               && Math.abs(swerve.getMyPose().getRotation().getDegrees() - targetPosition.getRotation().getDegrees()) < rotationTolerance;
    }
}
