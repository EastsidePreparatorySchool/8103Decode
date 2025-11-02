package org.firstinspires.ftc.teamcode.commandbase.safecommands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.unsafebasecommands.TransferStateCommand;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class TransferSetPositionCommand extends SequentialCommandGroup {
    public TransferSetPositionCommand(SpindexerSubsystem spindexerSubsystem, TransferSubsystem transferSubsystem, TransferSubsystem.TransferState state) {
        addRequirements(spindexerSubsystem, transferSubsystem);
        // If the spindexer is in any intake state, do not allow transfer commands to run
        SpindexerSubsystem.SpindexerState s = spindexerSubsystem.state;
        if (s == SpindexerSubsystem.SpindexerState.INTAKE_ONE ||
            s == SpindexerSubsystem.SpindexerState.INTAKE_TWO ||
            s == SpindexerSubsystem.SpindexerState.INTAKE_THREE) {
            return; // no-op
        }
        addCommands(
                new TransferStateCommand(transferSubsystem, state),
                new WaitCommand(300)
        );
    }

    public TransferSetPositionCommand(TransferSubsystem.TransferState state) {
        this(RobotHardware.getInstance().spindexerSubsystem,
             RobotHardware.getInstance().transferSubsystem,
             state);
    }
}
