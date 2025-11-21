package org.firstinspires.ftc.teamcode.commandbase.complexcommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commandbase.safecommands.TransferSetPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.unsafebasecommands.TransferStateCommand;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

/**
 * Transfers a ball up into the shooter, waits briefly, then retracts.
 * Slot bookkeeping and any follow-up logic should be handled by the calling OpMode.
 */
public class TransferAndShootCommand extends SequentialCommandGroup {

    public TransferAndShootCommand(TransferSubsystem transferSubsystem,
                                   ShooterSubsystem shooterSubsystem) {
        addRequirements(transferSubsystem, shooterSubsystem, RobotHardware.getInstance().spindexerSubsystem);

        addCommands(
                new WaitUntilCommand(shooterSubsystem::withinTolerance),
                new TransferSetPositionCommand(TransferSubsystem.TransferState.UP),
                new WaitCommand(250),
                new TransferStateCommand(transferSubsystem, TransferSubsystem.TransferState.DOWN)
        );
    }

    public TransferAndShootCommand() {
        this(RobotHardware.getInstance().transferSubsystem,
             RobotHardware.getInstance().shooterSubsystem);
    }
}
