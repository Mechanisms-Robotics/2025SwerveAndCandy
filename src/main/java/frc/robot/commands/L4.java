package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeMech;
import frc.robot.subsystems.Elevator;

public class L4 extends Command {
    private final Elevator m_elevator;
    private final Supplier<Boolean> clutch;
    private final Supplier<Boolean> up;
    private final Supplier<Boolean> down;

    /**
     * Raise the elevator to L4 and if the clutch is engaged, it raises a little heigher.
     * The purpose of this is to allow the coral to come all the way out of the mechanism as it gets stuck.
     * 
     * @param elevator used for raising the elevator.
     * @param clutch button boolean supplier for determining if the elevator is to offset a little higher.
     */
    public L4(Elevator elevator, Supplier<Boolean> clutch, Supplier<Boolean> up, Supplier<Boolean> down) {
        m_elevator = elevator;
        this.clutch = clutch;
        this.up = up;
        this.down = down;
        addRequirements(m_elevator);
    }

    public L4(Elevator elevator) {
        this(elevator, ()->false);
    }

    public L4(Elevator elevator, Supplier<Boolean> clutch) {
        this(elevator, clutch, ()->false, ()->false);
    }
    
    @Override
    public void execute() {
        if (clutch.get()) {
            if (up.get()) {
                m_elevator.increaseL4Offset(200);
            } else if (down.get()) {
                m_elevator.increaseL4Offset(-200);
            }
            m_elevator.setTargetPosition(Elevator.L4_Offset);
        } else {
            m_elevator.setTargetPosition(Elevator.L4);
        }
    }
}
