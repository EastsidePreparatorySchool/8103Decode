package org.firstinspires.ftc.teamcode.commandbase.safecommands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.unsafebasecommands.SpindexerStateCommand;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class SpindexerCommand extends SequentialCommandGroup {
    public SpindexerCommand(SpindexerSubsystem spindexerSubsystem, TransferSubsystem transferSubsystem, SpindexerSubsystem.SpindexerState state) {
        addRequirements(spindexerSubsystem, transferSubsystem);
        if(transferSubsystem.state != TransferSubsystem.TransferState.UP) {
            addCommands(
                    new SpindexerStateCommand(spindexerSubsystem, state),
                    new WaitCommand(750)
            );
        }
    }
}
