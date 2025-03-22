package frc.robot.commands.swervedrive.auto;


import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class BargeAlign extends Command {
    private final SwerveSubsystem swerve;
    private final DoubleSupplier horizontalInput;
    private final PIDController rotationController = new PIDController(0.05, 0, 0);
    private final PIDController forwardBackController = new PIDController(1.0, 0, 0);


    public BargeAlign(SwerveSubsystem swerve, DoubleSupplier horizontalInput) {
        this.swerve = swerve;
        this.horizontalInput = horizontalInput;
        rotationController.enableContinuousInput(-180.0, 180.0);
        addRequirements(this.swerve);
    }

    @Override
    public void execute() {
        double redBargeDistance = Math.abs(swerve.getPose().getX() - Constants.FieldConstants.RED_BARG_POSE);
        double blueBargeDistance = Math.abs(swerve.getPose().getX() - Constants.FieldConstants.BLUE_BARG_POSE);
        double targetForwardBack = (blueBargeDistance < redBargeDistance) ? Constants.FieldConstants.BLUE_BARG_POSE : Constants.FieldConstants.RED_BARG_POSE;
        double targetRotation = (blueBargeDistance < redBargeDistance) ? 180.0 : 0.0;

        double horizontalOutput = Math.pow(horizontalInput.getAsDouble(), 3.0) * swerve.getMaximumChassisVelocity();
        double forwardBackOutput = forwardBackController.calculate(swerve.getPose().getX(), targetForwardBack);
        double rotationalOutput = rotationController.calculate(swerve.getHeading().getDegrees(), targetRotation);
        ChassisSpeeds fieldOrientedChassisSpeeds = new ChassisSpeeds(forwardBackOutput, horizontalOutput, rotationalOutput);
        swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(fieldOrientedChassisSpeeds, swerve.getMyPose().getRotation()));
    }
}
