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
     * Raise the elevator to L4 and if the clutch is engaged, it raises a little heigher.
     * The purpose of this is to allow the coral to come all the way out of the mechanism as it gets stuck.
     * 
     * @param elevator used for raising the elevator.
     * @param clutch button boolean supplier for determining if the elevator is to offset a little higher.
     */
    public ElevatorRest(Elevator elevator, AlgaeMech algaeMech, Supplier<Boolean> clutch) {
        m_elevator = elevator;
        m_algaeMech = algaeMech;
        this.clutch = clutch;
        addRequirements(m_elevator);
    }

    public ElevatorRest(Elevator elevator, AlgaeMech algaeMech) {
        this(elevator, algaeMech, ()->false);
    }

    @Override
    public void execute() {
        if (clutch.get()) {
            m_elevator.setTargetPosition(Elevator.RESTING);
            m_algaeMech.setWristAngle(AlgaeMech.WRIST_INTAKE);
            m_algaeMech.groundIntake();
        } else {
            m_elevator.setTargetPosition(Elevator.RESTING);
        }
    }

}
