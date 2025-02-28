package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeMech;
import frc.robot.subsystems.Elevator;

public class L2 extends Command {
    private final Elevator m_elevator;

    private final AlgaeMech m_algaeMech;
    private final Supplier<Boolean> clutch;

    /**
     * Raise the elevator to the L2 position.
     * If the clutch is engaged, the elevator will raise a little heigher and the algae arms
     * will angle down to grab the algae.
     * 
     * @param elevator used for raising the elevator to L2
     * @param algaeMech used for angleing the algae mechanism when grabbing algae
     * @param clutch button boolean supplier for determining if it is in algae mode
     */
    public L2(Elevator elevator, AlgaeMech algaeMech, Supplier<Boolean> clutch) {
        m_elevator = elevator;
        m_algaeMech = algaeMech;
        this.clutch = clutch;
        addRequirements(elevator, algaeMech);
    }

    @Override
    public void execute() {
        if (clutch.get()) {
            m_elevator.setTargetPosition(Elevator.L2_ALGAE_OFFSET);
            m_algaeMech.setWristAngle(AlgaeMech.WRIST_ALGAE_PICKUP_ANGLE);
        } else {
            m_elevator.setTargetPosition(Elevator.L2);
        }
    }
}
