package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeMech;
import frc.robot.subsystems.Elevator;

public class L2 extends Command {
    private final Elevator m_elevator;

    private final AlgaeMech m_algaeMech;
    private final Supplier<Boolean> algae_clutch;
    private final Supplier<Boolean> coral_clutch;
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
    public L2(Elevator elevator, AlgaeMech algaeMech, Supplier<Boolean> algae_clutch, Supplier<Boolean> coral_clutch, Supplier<Boolean> up, Supplier<Boolean> down) {
        m_elevator = elevator;
        m_algaeMech = algaeMech;
        this.algae_clutch = algae_clutch;
        this.coral_clutch = coral_clutch;
        this.up = up;
        this.down = down;
        addRequirements(elevator, algaeMech);
    }

    public L2(Elevator elevator, AlgaeMech algaeMech, Supplier<Boolean> algae_clutch, Supplier<Boolean> coral_clutch) {
        this(elevator, algaeMech, algae_clutch, coral_clutch, ()->false, ()->false);
    }

    public L2(Elevator elevator, AlgaeMech algaeMech) {
        this(elevator, algaeMech, ()->false, ()->false);
    }

    @Override
    public void execute() {
        if (algae_clutch.get()) {
            m_elevator.setTargetPosition(Elevator.L2_ALGAE_OFFSET);
            m_algaeMech.setWristAngle(AlgaeMech.WRIST_ANGLE_DOWN);
            if (up.get()) {
                m_elevator.increaseL2AlgaeOffset(200);
            } else if (down.get()) {
                m_elevator.increaseL2AlgaeOffset(-200);
            }
        } else if (coral_clutch.get()) {
            m_elevator.setTargetPosition(Elevator.L2_Offset);
            if (up.get()) {
                m_elevator.increaseL2Offset(200);
            } else if (down.get()) {
                m_elevator.increaseL2Offset(-200);
            }
        } else {
            m_elevator.setTargetPosition(Elevator.L2);
        }
    }
}
