package org.firstinspires.ftc.teamcode.commandbase.complexcommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.safecommands.TransferSetPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.unsafebasecommands.TransferStateCommand;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

/**
 * Transfers a ball up into the shooter, waits briefly, then retracts.
 * If the shooter was running during this command, invokes the provided callback to
 * mark the current spindexer slot as empty (slot bookkeeping is handled by the caller/opmode).
 */
public class TransferAndShootCommand extends SequentialCommandGroup {
    public interface ShotCallback {
        void onShotFired();
    }

    public TransferAndShootCommand(TransferSubsystem transferSubsystem,
                                   ShooterSubsystem shooterSubsystem,
                                   ShotCallback callback) {
        addRequirements(transferSubsystem, shooterSubsystem, RobotHardware.getInstance().spindexerSubsystem);

        boolean shooterRunningAtStart = shooterSubsystem.targetRpm > 0.0;

        addCommands(
                new TransferSetPositionCommand(TransferSubsystem.TransferState.UP),
                new WaitCommand(50),
                // Use unsafe immediate drop to avoid an additional built-in wait
                new TransferStateCommand(transferSubsystem, TransferSubsystem.TransferState.DOWN),
                new InstantCommand(() -> {
                    if (shooterRunningAtStart && callback != null) {
                        callback.onShotFired();
                    }
                })
        );
    }

    public TransferAndShootCommand(ShotCallback callback) {
        this(RobotHardware.getInstance().transferSubsystem,
             RobotHardware.getInstance().shooterSubsystem,
             callback);
    }
}
