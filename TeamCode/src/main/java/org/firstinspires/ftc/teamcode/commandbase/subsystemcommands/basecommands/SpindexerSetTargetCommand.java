package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.basecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class SpindexerSetTargetCommand extends InstantCommand {
    public SpindexerSetTargetCommand(SpindexerSubsystem spindexerSubsystem, double targetDegrees) {
        super(() -> spindexerSubsystem.setTargetDegrees(targetDegrees));
    }
}
