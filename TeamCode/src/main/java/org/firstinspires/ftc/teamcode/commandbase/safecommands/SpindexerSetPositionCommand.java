package org.firstinspires.ftc.teamcode.commandbase.safecommands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.unsafebasecommands.SpindexerStateCommand;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class SpindexerSetPositionCommand extends SequentialCommandGroup {
    public SpindexerSetPositionCommand(SpindexerSubsystem spindexerSubsystem, TransferSubsystem transferSubsystem, SpindexerSubsystem.SpindexerState state) {
        addRequirements(spindexerSubsystem, transferSubsystem);
        if(transferSubsystem.state == TransferSubsystem.TransferState.DOWN) {
            addCommands(
                    new SpindexerStateCommand(spindexerSubsystem, state),
                    new WaitCommand(750)
            );
        }
    }

    public SpindexerSetPositionCommand(SpindexerSubsystem.SpindexerState state) {
        this(RobotHardware.getInstance().spindexerSubsystem,
             RobotHardware.getInstance().transferSubsystem,
             state);
    }
}
