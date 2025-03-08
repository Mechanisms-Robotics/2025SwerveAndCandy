package frc.robot.commands.swervedrive.auto;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.LimeLight.ApriltagData;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * Allows the drive to drive as usualy except, if the limelight sees the reef it will point to it.
 * The Robot will point toward the closest reef april tag.
 * This command does not use any odometry or localisation.
 */
public class DriveWhileApriltagPoint extends Command {
    private final SwerveSubsystem swerve;
    private final LimeLight limelight;
    private final Supplier<Double> xVal;
    private final Supplier<Double> yVal;
    private final Supplier<Double> rVal;
    private final PIDController pidController = new PIDController(0.5, 0.0, 0.0);
    private final int[] apriltags;

    /**
     * Allows the drive to drive as usualy except, if the limelight sees the reef it will point to it.
     * 
     * @param swerve
     * @param limelight used for getting the yaw of the reef apriltags
     * @param xVal x component of the joystick
     * @param yVal y component of the joystick
     * @param rVal rotational component of the joystick, used if no apriltag on the reef was found
     * @param apriltags int list of all the the april tags the robot will try to lock onto, it will lock onto the closest one
     */
    public DriveWhileApriltagPoint(SwerveSubsystem swerve, LimeLight limelight, 
                             Supplier<Double> xVal, Supplier<Double> yVal, Supplier<Double> rVal, int[] apriltags) {
        this.swerve = swerve;
        this.limelight = limelight;
        this.xVal = xVal;
        this.yVal = yVal;
        this.rVal = rVal;
        pidController.setSetpoint(0.0);
        // the yaw of the apriltag is inbetween -180 and 180
        pidController.enableContinuousInput(-180, 180);

        this.apriltags = apriltags;

        addRequirements(this.swerve, this.limelight);
    }

    @Override
    public void execute() {
        Optional<ApriltagData> apriltag = limelight.getClosestAprilTag(apriltags);

        SwerveInputStream driveAngularVelocity;
        if (apriltag.isPresent()) {
            ApriltagData tag = apriltag.get();

            // apply the sigmoid curve, or s shape curve to smooth the pid values to be between 1 and -1
            double rotationalOutput = 1 / (1 + Math.exp(-pidController.calculate(tag.getYaw())));
            driveAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(),
                () -> Math.pow(yVal.get(), 3),
                () -> Math.pow(xVal.get(), 3))
                .withControllerRotationAxis(() -> rotationalOutput)
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true);
        } else {
            driveAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(),
                () -> Math.pow(yVal.get(), 3),
                () -> Math.pow(xVal.get(), 3))
                .withControllerRotationAxis(() -> -Math.signum(rVal.get())*Math.pow(rVal.get(), 2))
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true);
        
        }
        swerve.driveFieldOriented(driveAngularVelocity.get());
    }
}
