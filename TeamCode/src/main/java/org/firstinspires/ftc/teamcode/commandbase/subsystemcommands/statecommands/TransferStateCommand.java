package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.statecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class TransferStateCommand extends InstantCommand {
    public TransferStateCommand(TransferSubsystem.TransferState transferState) {
        super (
                () -> RobotHardware.getInstance().transferSubsystem.setTransferState(transferState)
        );
    }

}

