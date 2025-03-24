package frc.robot.commands.swervedrive.auto;


import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class BargeAlign extends Command {
    private final SwerveSubsystem swerve;
    private final DoubleSupplier horizontalInput;
    private final PIDController rotationController = new PIDController(0.07, 0, 0);
    private final PIDController forwardBackController = new PIDController(3.0, 0, 0);
    private final double maxTranslationComponent = 1.0; // m/s
    private final double maxRotationVelocity = Math.PI; // m/s


    public BargeAlign(SwerveSubsystem swerve, DoubleSupplier horizontalInput) {
        this.swerve = swerve;
        this.horizontalInput = horizontalInput;
        rotationController.enableContinuousInput(-180.0, 180.0);
        SmartDashboard.putData("Commands/BargeAlign/forward back controller", forwardBackController);
        SmartDashboard.putData("Commands/BargeAlign/rotation controller", rotationController);
        addRequirements(this.swerve);
    }

    @Override
    public void execute() {
        double targetForwardBack;
        double targetRotation;
        if (swerve.getMyPose().getX() < FieldConstants.FIELD_LENGTH/2) {
            targetRotation = 0.0;
            targetForwardBack = FieldConstants.BLUE_BARG_POSE;
        } else {
            targetRotation = 180.0;
            targetForwardBack = FieldConstants.RED_BARG_POSE;
        }

        double horizontalOutput = Math.pow(horizontalInput.getAsDouble(), 3.0) * swerve.getMaximumChassisVelocity();
        double forwardBackOutput = forwardBackController.calculate(swerve.getMyPose().getX(), targetForwardBack);
        double rotationalOutput = rotationController.calculate(swerve.getMyPose().getRotation().getDegrees(), targetRotation);
        forwardBackOutput = MathUtil.clamp(forwardBackOutput, -maxTranslationComponent, maxTranslationComponent);
        rotationalOutput = MathUtil.clamp(rotationalOutput, -maxRotationVelocity, maxRotationVelocity);
        ChassisSpeeds fieldOrientedChassisSpeeds = new ChassisSpeeds(forwardBackOutput, horizontalOutput, rotationalOutput);
        swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(fieldOrientedChassisSpeeds, swerve.getMyPose().getRotation()));
    }
}
