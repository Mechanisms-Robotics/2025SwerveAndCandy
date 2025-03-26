package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeMech;
import frc.robot.subsystems.Elevator;

public class L3 extends Command {
    private final Elevator m_elevator;

    private final AlgaeMech m_algaeMech;
    private final Supplier<Boolean> algae_clutch;
    private final Supplier<Boolean> coral_clutch;
    private final Supplier<Boolean> up;
    private final Supplier<Boolean> down;

    /**
     * Raise the elevator to the L3 position.
     * If the clutch is engaged, the elevator will raise a little heigher and the algae arms
     * will angle down to grab the algae.
     * 
     * @param elevator used for raising the elevator to L2
     * @param algaeMech used for angleing the algae mechanism when grabbing algae
     * @param algae_clutch button boolean supplier for determining if it is in algae mode
     * @param coral_clutch button boolean supplier for determining if it is in coral mode and needing offsets
     * @param up button boolean supplier for raising elevator offset position
     * @param down button boolean supplier for lowering elevator offset position
     */
    public L3(Elevator elevator, AlgaeMech algaeMech, Supplier<Boolean> algae_clutch, Supplier<Boolean> coral_clutch, Supplier<Boolean> up, Supplier<Boolean> down) {
        m_elevator = elevator;
        m_algaeMech = algaeMech;
        this.algae_clutch = algae_clutch;
        this.coral_clutch = coral_clutch;
        this.up = up;
        this.down = down;
        addRequirements(elevator, algaeMech);
    }

     /**
      * Raise the elevator to the L3 position.
      * If the clutch is engaged, the elevator will raise a little heigher and the algae arms
      * will angle down to grab the algae.
      * 
      * @param elevator used for raising the elevator to L2
      * @param algaeMech used for angleing the algae mechanism when grabbing algae
      * @param algae_clutch button boolean supplier for determining if it is in algae mode
      * @param coral_clutch button boolean supplier for determining if it is in algae mode
      */
    public L3(Elevator elevator, AlgaeMech algaeMech, Supplier<Boolean> algae_clutch, Supplier<Boolean> coral_clutch) {
        this(elevator, algaeMech, algae_clutch, coral_clutch, ()->false, ()->false);
    }

    @Override
    public void execute() {
        if (algae_clutch.get()) {
            m_elevator.setTargetPosition(Elevator.L3_ALGAE_OFFSET);
            m_algaeMech.setWristAngle(AlgaeMech.WRIST_ANGLE_DOWN);
            if (up.get()) {
                m_elevator.increaseL3AlgaeOffset(200);
            } else if (down.get()) {
                m_elevator.increaseL3AlgaeOffset(-200);
            }
        } else if (coral_clutch.get()){
            m_elevator.setTargetPosition(Elevator.L3_Offset);
            if (up.get()) {
                m_elevator.increaseL3Offset(200);
            } else if (down.get()) {
                m_elevator.increaseL3Offset(-200);
            }
        } else {
            m_elevator.setTargetPosition(Elevator.L3);
        }
    }
}
