package org.firstinspires.ftc.teamcode.commandbase.complexcommands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.safecommands.SpindexerSetPositionCommand;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

/**
 * Switches spindexer to OUTTAKE slots 1, 2, 3 and performs a transfer+shoot for each.
 */
public class TripleShotCommand extends SequentialCommandGroup {
    public interface SlotEmptiedListener {
        void onSlotEmptied(int slotIdx); // 0,1,2
    }

    private static SpindexerSubsystem.SpindexerState outtakeStateForSlot(int slotIdx) {
        switch (slotIdx) {
            case 0: return SpindexerSubsystem.SpindexerState.OUTTAKE_ONE;
            case 1: return SpindexerSubsystem.SpindexerState.OUTTAKE_TWO;
            case 2: return SpindexerSubsystem.SpindexerState.OUTTAKE_THREE;
        }
        return SpindexerSubsystem.SpindexerState.OUTTAKE_ONE;
    }

    public TripleShotCommand(SpindexerSubsystem spindexer,
                             TransferSubsystem transfer,
                             ShooterSubsystem shooter,
                             SlotEmptiedListener listener) {
        addRequirements(spindexer, transfer, shooter);

        for (int i = 0; i < 3; i++) {
            final int slotIdx = i;
            addCommands(
                    new SpindexerSetPositionCommand(outtakeStateForSlot(slotIdx)),
                    new TransferAndShootCommand(() -> {
                        if (listener != null) listener.onSlotEmptied(slotIdx);
                    }),
                    new WaitCommand(2000)
            );
        }

        // After triple shot, return to slot 1 intaking
        addCommands(new SpindexerSetPositionCommand(SpindexerSubsystem.SpindexerState.INTAKE_ONE));
    }

    public TripleShotCommand(SlotEmptiedListener listener) {
        this(RobotHardware.getInstance().spindexerSubsystem,
             RobotHardware.getInstance().transferSubsystem,
             RobotHardware.getInstance().shooterSubsystem,
             listener);
    }
}
