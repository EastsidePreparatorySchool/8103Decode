package org.firstinspires.ftc.teamcode.commandbase.unsafebasecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShooterStateCommand extends InstantCommand {
    public ShooterStateCommand(ShooterSubsystem shooterSubsystem, ShooterSubsystem.ShooterState shooterState) {
        super(() -> shooterSubsystem.setShooterState(shooterState));
    }

    public ShooterStateCommand(ShooterSubsystem.ShooterState shooterState) {
        this(RobotHardware.getInstance().shooterSubsystem, shooterState);
    }
}
