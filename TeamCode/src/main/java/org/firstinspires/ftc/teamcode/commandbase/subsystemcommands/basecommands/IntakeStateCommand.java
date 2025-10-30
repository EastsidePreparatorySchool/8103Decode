package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.basecommands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeStateCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final IntakeSubsystem.IntakeState intakeState;

    public IntakeStateCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.IntakeState intakeState) {
        this.intakeSubsystem = intakeSubsystem;
        this.intakeState = intakeState;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setIntakeState(intakeState);
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
