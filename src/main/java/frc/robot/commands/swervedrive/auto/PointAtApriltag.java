package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.LimeLight.ApriltagData;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PointAtApriltag extends Command {
    private final SwerveSubsystem swerve;
    private final LimeLight limeLight;
    private final ApriltagData apriltag;
    private final PIDController pidController;

    /**
     * Point at an apriltag
     * 
     * @param swerve used for angling the robot at the apriltag
     * @param limeLight used for finding the yaw of the apriltag
     * @param id id of the april tag being referenced
     * @param targetAngle the target yaw in degrees between the center of the camera and the center of the apriltag
     * @param errorTollerance tollerance of the difference of of the target angle and the actual angle
     * @param pidController used for controlling the swerves direction
     */
    public PointAtApriltag(SwerveSubsystem swerve, LimeLight limeLight, int id, double targetAngle, double errorTollerance, PIDController pidController) {
        this.swerve = swerve;
        this.limeLight = limeLight;
        apriltag = limeLight.getApriltag(id);
        this.pidController = pidController;
        pidController.setTolerance(Units.degreesToRadians(errorTollerance));
        pidController.setSetpoint(targetAngle);

        addRequirements(this.swerve, this.limeLight);
    }

    /**
     * Point at an apriltag with default tollerance and default pid controller
     * 
     * @param swerve used for angling the robot at the apriltag
     * @param limeLight used for finding the yaw of the apriltag
     * @param id id of the april tag being referenced
     */
    public PointAtApriltag(SwerveSubsystem swerve, LimeLight limeLight, int id) {
        this(swerve, limeLight, id, 0.0, 0.1, new PIDController(0.01, 0.0, 0.0));
    }

    /**
     * Point at an apriltag with with default tollerance and default pid controller
     * 
     * @param swerve used for angling the robot at the apriltag
     * @param limeLight used for finding the yaw of the apriltag
     * @param id id of the april tag being referenced
     * @param targetAngle the desired yaw angle between the center of the camera and the center of the apriltag
     */
    public PointAtApriltag(SwerveSubsystem swerve, LimeLight limeLight, int id, double targetAngle) {
        this(swerve, limeLight, id, targetAngle, 1, new PIDController(0.01, 0.0, 0.0));
    }

    @Override
    public void execute() {
        double output;
        // only drive if the apriltag is detected
        if (apriltag.getDetected()) {
            output = pidController.calculate(apriltag.getYaw());
        } else {
            output = 0.0;
        }

        swerve.drive(new Translation2d(), output, false);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}
