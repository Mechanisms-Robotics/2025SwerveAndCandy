package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeMech;
import frc.robot.subsystems.Elevator;

public class ElevatorRest extends Command {
    private final Elevator m_elevator;
    private final AlgaeMech m_algaeMech;
   
    /**
     * 
     * @param elevator used for raising the elevator.
     * @param algaeMech used for setting the wrist angle.
     */
    public ElevatorRest(Elevator elevator, AlgaeMech algaeMech) {
        m_elevator = elevator;
        m_algaeMech = algaeMech;
        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        // [M.Fox] After speaking with Leif and Ada, determined we don't need to examine clutch
        // for this command. Just set wrist angle for Intake every time before going to RESTING.
        m_algaeMech.setWristAngle(AlgaeMech.WRIST_INTAKE);
        m_elevator.setTargetPosition(Elevator.RESTING);

        // [M.Fox] This command was causing intake to spin continuously until Leif pressed some other button
        // to make it stop. Since Leif has intake/outtake controls on this controller anyway, he
        // asked to remove this from the elevator commands driven by secondary controller.
       // m_algaeMech.groundIntake();
         
    }

}
