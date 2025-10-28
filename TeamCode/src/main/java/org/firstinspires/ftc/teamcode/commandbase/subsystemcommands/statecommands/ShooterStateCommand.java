package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.statecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShooterStateCommand extends InstantCommand {
    public ShooterStateCommand(ShooterSubsystem.ShooterState shooterState) {
        super (
                () -> RobotHardware.getInstance().shooterSubsystem.setShooterState(shooterState)
        );
    }
}
