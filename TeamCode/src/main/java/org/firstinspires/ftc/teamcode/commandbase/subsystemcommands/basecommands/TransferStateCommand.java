package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.basecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class TransferStateCommand extends InstantCommand {
    public TransferStateCommand(TransferSubsystem transferSubsystem, TransferSubsystem.TransferState transferState) {
        super(() -> transferSubsystem.setTransferState(transferState));
    }
}
