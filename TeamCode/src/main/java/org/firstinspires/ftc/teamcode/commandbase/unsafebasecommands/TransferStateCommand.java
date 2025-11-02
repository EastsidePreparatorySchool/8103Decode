package org.firstinspires.ftc.teamcode.commandbase.unsafebasecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class TransferStateCommand extends InstantCommand {
    public TransferStateCommand(TransferSubsystem transferSubsystem, TransferSubsystem.TransferState transferState) {
        super(() -> {
            transferSubsystem.setTransferState(transferState);
        });
        addRequirements(transferSubsystem);
    }

    public TransferStateCommand(TransferSubsystem.TransferState transferState) {
        this(RobotHardware.getInstance().transferSubsystem, transferState);
    }
}
