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
 * Any bookkeeping (e.g., marking slots empty) should be handled in the calling OpMode.
 */
public class TripleShotCommand extends SequentialCommandGroup {

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
                             ShooterSubsystem shooter) {
        addRequirements(spindexer, transfer, shooter);

        for (int i = 0; i < 3; i++) {
            final int slotIdx = i;
            addCommands(
                    new SpindexerSetPositionCommand(outtakeStateForSlot(slotIdx)),
                    new TransferAndShootCommand(),
                    new WaitCommand(2000)
            );
        }

        // After triple shot, return to slot 1 intaking
        addCommands(new SpindexerSetPositionCommand(SpindexerSubsystem.SpindexerState.INTAKE_ONE));
    }

    public TripleShotCommand() {
        this(RobotHardware.getInstance().spindexerSubsystem,
             RobotHardware.getInstance().transferSubsystem,
             RobotHardware.getInstance().shooterSubsystem);
    }
}
