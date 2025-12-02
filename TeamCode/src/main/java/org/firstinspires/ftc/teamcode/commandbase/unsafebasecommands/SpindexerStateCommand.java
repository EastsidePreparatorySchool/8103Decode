package org.firstinspires.ftc.teamcode.commandbase.unsafebasecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class SpindexerStateCommand extends InstantCommand {
    public SpindexerStateCommand(SpindexerSubsystem spindexerSubsystem, SpindexerSubsystem.SpindexerState spindexerState) {
        super(() -> spindexerSubsystem.setSpindexerState(spindexerState));
        addRequirements(spindexerSubsystem);
    }

    public SpindexerStateCommand(SpindexerSubsystem.SpindexerState spindexerState) {
        this(RobotHardware.getInstance().spindexerSubsystem, spindexerState);
    }
}
