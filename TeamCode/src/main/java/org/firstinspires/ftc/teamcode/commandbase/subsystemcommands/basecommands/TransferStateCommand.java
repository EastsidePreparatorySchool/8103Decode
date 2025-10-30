package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.basecommands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class TransferStateCommand extends CommandBase {
    private final TransferSubsystem transferSubsystem;
    private final TransferSubsystem.TransferState transferState;

    public TransferStateCommand(TransferSubsystem transferSubsystem, TransferSubsystem.TransferState transferState) {
        this.transferSubsystem = transferSubsystem;
        this.transferState = transferState;
        addRequirements(transferSubsystem);
    }

    @Override
    public void initialize() {
        transferSubsystem.setTransferState(transferState);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public void runNow() {
        initialize();
        end(false);
    }
}

