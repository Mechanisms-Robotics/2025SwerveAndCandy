package frc.robot.commands.swervedrive.auto;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.LimeLight.ApriltagData;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class NoLocoBigApriltagReef extends SequentialCommandGroup {
    private final SwerveSubsystem swerve;
    private final LimeLight limelight;

    public NoLocoBigApriltagReef(SwerveSubsystem swerve, LimeLight limelight, BooleanSupplier right) {
        this.swerve = swerve;
        this.limelight = limelight;

        addRequirements(this.swerve);
        addCommands(new DeferredCommand(() -> new DriveTransform(swerve, findReefTransform(right.getAsBoolean())), getRequirements()));
    }

    public Transform2d findReefTransform(boolean right) {
        Optional<ApriltagData> apriltag = limelight.getClosestReefApriltag();
        if (apriltag.isPresent()) {
            return apriltag.get().getRobotApriltagTransform();
        }
        return Transform2d.kZero;
    }
}
