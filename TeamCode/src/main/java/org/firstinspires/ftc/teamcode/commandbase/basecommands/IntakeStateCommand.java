package org.firstinspires.ftc.teamcode.commandbase.basecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeStateCommand extends InstantCommand {
    public IntakeStateCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.IntakeState intakeState) {
        super(() -> intakeSubsystem.setIntakeState(intakeState));
    }

    public IntakeStateCommand(IntakeSubsystem.IntakeState intakeState) {
        this(RobotHardware.getInstance().intakeSubsystem, intakeState);
    }
}
