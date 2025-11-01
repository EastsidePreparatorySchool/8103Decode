package org.firstinspires.ftc.teamcode.commandbase.safecommands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.unsafebasecommands.TransferStateCommand;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(SpindexerSubsystem spindexerSubsystem, TransferSubsystem transferSubsystem, TransferSubsystem.TransferState state) {
        addRequirements(spindexerSubsystem, transferSubsystem);
        addCommands(
                new TransferStateCommand(transferSubsystem, state),
                new WaitCommand(300)
        );
    }
}
