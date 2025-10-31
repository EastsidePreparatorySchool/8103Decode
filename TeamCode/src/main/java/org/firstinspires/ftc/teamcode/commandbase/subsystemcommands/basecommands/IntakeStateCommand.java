package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.basecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeStateCommand extends InstantCommand {
    public IntakeStateCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.IntakeState intakeState) {
        super(() -> intakeSubsystem.setIntakeState(intakeState));
    }
}
