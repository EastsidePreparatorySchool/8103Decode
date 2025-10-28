package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.statecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class SpindexerStateCommand extends InstantCommand {
    public SpindexerStateCommand(SpindexerSubsystem.SpindexerState spindexerState) {
        super(
                () -> RobotHardware.getInstance().spindexerSubsystem.setSpindexerState(spindexerState)
        );
    }
}
