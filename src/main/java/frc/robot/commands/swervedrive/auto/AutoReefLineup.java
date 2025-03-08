package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoReefLineup extends Command {
    private final SwerveSubsystem swerve;
    private final LimeLight limeLight;
    
    public AutoReefLineup(SwerveSubsystem swerve, LimeLight limeLight1) {
        this.swerve = swerve;
        this.limeLight = limeLight1;

        addRequirements(this.swerve, this.limeLight);
    }
}
