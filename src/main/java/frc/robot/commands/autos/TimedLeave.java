package frc.robot.commands.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TimedLeave extends Command {
    private final SwerveSubsystem m_swerve;
    private final Timer m_timer = new Timer();
    private final double m_time;
    private final double speed;

    private static final double MPS = -0.5;
    private static final double SECONDS = 5.0;

    /**
     * Leave the starting zone, go forward for time seconds at speed meters per second
     * 
     * @param swerve my dog named jeff
     * @param time time in seconds to go forward
     * @param speed speed in meters per second to go forward
     */
    public TimedLeave(SwerveSubsystem swerve, double time, double speed) {
        m_swerve = swerve;
        m_time = time;
        this.speed = speed;
        addRequirements(m_swerve);
    }

    /**
     * Leave the starting zone for time number of seconds
     * 
     * @param swerve ghengo khango
     * @param time time in seconds to go forward
     */
    public TimedLeave(SwerveSubsystem swerve, double time) {
        this(swerve, time, MPS);
    }

    /**
     * Leave the field by goring forward for a set number of seconds at a set speed
     * 
     * @param swerve kyle the koala
     */
    public TimedLeave(SwerveSubsystem swerve) {
        this(swerve, SECONDS);
    }

    @Override
    public void initialize() {
        m_timer.start();
    }
    @Override
    public void execute() {
        m_swerve.drive(new ChassisSpeeds(speed, 0, 0));
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_timer.reset();
        m_swerve.drive(new ChassisSpeeds(0, 0, 0));
    }
    
    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_time);
    }
}
