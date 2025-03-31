package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swervedrive.auto.PIDtoPosition;
import frc.robot.subsystems.LimeLight.ApriltagData;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.State;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class LEDCommand extends Command {
    private final SwerveSubsystem swerve;
    private final StateMachine stateMachine;

    public LEDCommand(SwerveSubsystem swerve, StateMachine stateMachine) {
        this.swerve = swerve;
        this.stateMachine = stateMachine;
        // requires the state machine because it is a default command
        addRequirements(stateMachine);
    }
    
    @Override
    public void execute() {
        if (false) {
            stateMachine.setState(State.DrivingToReefTarget);
        } else {
            if (ApriltagData.validPositionMeasurement()) {
                stateMachine.setState(State.DetectedReefTarget);
            } else {
                stateMachine.setState(State.NoTargetIdentified);   
            }
        }
    }
}
