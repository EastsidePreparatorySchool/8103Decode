package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.basecommands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class SpindexerStateCommand extends CommandBase {
    private final SpindexerSubsystem spindexerSubsystem;
    private final SpindexerSubsystem.SpindexerState spindexerState;

    public SpindexerStateCommand(SpindexerSubsystem spindexerSubsystem, SpindexerSubsystem.SpindexerState spindexerState) {
        this.spindexerSubsystem = spindexerSubsystem;
        this.spindexerState = spindexerState;
        addRequirements(spindexerSubsystem);
    }

    @Override
    public void initialize() {
        spindexerSubsystem.setSpindexerState(spindexerState);
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
