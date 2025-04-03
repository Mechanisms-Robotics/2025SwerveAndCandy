package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeMech;
import frc.robot.subsystems.Elevator;

public class ElevatorBarge extends Command {
        private final Elevator m_elevator;

    private final AlgaeMech m_algaeMech;
    private final Supplier<Boolean> up;
    private final Supplier<Boolean> down;

    /**
     * Raise the elevator to the L2 position.
     * If the clutch is engaged, the elevator will raise a little heigher and the algae arms
     * will angle down to grab the algae.
     * 
     * @param elevator used for raising the elevator to L2
     * @param algaeMech used for angleing the algae mechanism when grabbing algae
     * @param clutch button boolean supplier for determining if it is in algae mode
     */
    public ElevatorBarge(Elevator elevator, AlgaeMech algaeMech, Supplier<Boolean> up, Supplier<Boolean> down) {
        m_elevator = elevator;
        m_algaeMech = algaeMech;
        this.up = up;
        this.down = down;
        addRequirements(elevator, algaeMech);
    }

    public ElevatorBarge(Elevator elevator, AlgaeMech algaeMech) {
        this(elevator, algaeMech, ()->false, ()->false);
    }

    @Override
    public void initialize() {
        m_algaeMech.setWristAngle(AlgaeMech.WRIST_BARGE);
    }

    @Override
    public void execute() {
        m_elevator.setTargetPosition(Elevator.BARGE);
    }
}
