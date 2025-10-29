package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.statecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeStateCommand extends InstantCommand {
    public IntakeStateCommand(IntakeSubsystem.IntakeState intakeState) {
        super (
                () -> RobotHardware.getInstance().intakeSubsystem.setIntakeState(intakeState)
        );
    }
}
