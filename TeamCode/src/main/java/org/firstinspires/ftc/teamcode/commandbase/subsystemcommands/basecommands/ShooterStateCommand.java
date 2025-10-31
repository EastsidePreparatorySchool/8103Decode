package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.basecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShooterStateCommand extends InstantCommand {
    public ShooterStateCommand(ShooterSubsystem shooterSubsystem, ShooterSubsystem.ShooterState shooterState) {
        super(() -> shooterSubsystem.setShooterState(shooterState));
    }
}
