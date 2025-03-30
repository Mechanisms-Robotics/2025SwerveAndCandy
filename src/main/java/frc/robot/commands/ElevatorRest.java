package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeMech;
import frc.robot.subsystems.Elevator;

public class ElevatorRest extends Command {
    private final Elevator m_elevator;
    private final AlgaeMech m_algaeMech;
    private final Supplier<Boolean> clutch;
   
    /**
     * 
     * @param elevator used for raising the elevator.
     * @param algaeMech used for setting the wrist angle.
     */
    public ElevatorRest(Elevator elevator, AlgaeMech algaeMech, Supplier<Boolean> clutch) {
        m_elevator = elevator;
        m_algaeMech = algaeMech;
        this.clutch = clutch;
        addRequirements(m_elevator, algaeMech);
    }

    public ElevatorRest(Elevator elevator, AlgaeMech algaeMech) {
        this(elevator, algaeMech, ()->false);
    }

    @Override
    public void execute() {

        // TODO -- DURING CODE REVIEW: Program Team decided to remove the wrist angle adjustment until further
        // discussion with Drive TEam.

        // This is the code to use if Drive Team agrees they want clutch operation to move wrist at rest
        if (clutch.get()) {
            m_algaeMech.setWristAngle(AlgaeMech.WRIST_INTAKE);
        } else {
            // clutch not engaged, thus don't move the wrist
            // i.e, DO NOTHING
        }

        m_elevator.setTargetPosition(Elevator.RESTING);

        // [M.Fox] This command was causing intake to spin continuously until Leif pressed some other button
        // to make it stop. Since Leif has intake/outtake controls on this controller anyway, he
        // asked to remove this from the elevator commands driven by secondary controller.
       // m_algaeMech.groundIntake();
         
    }

}
