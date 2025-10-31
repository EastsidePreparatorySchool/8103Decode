package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.basecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class SpindexerStateCommand extends InstantCommand {
    public SpindexerStateCommand(SpindexerSubsystem spindexerSubsystem, SpindexerSubsystem.SpindexerState spindexerState) {
        super(() -> spindexerSubsystem.setSpindexerState(spindexerState));
    }
}
