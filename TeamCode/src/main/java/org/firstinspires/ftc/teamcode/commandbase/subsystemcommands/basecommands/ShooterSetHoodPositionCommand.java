package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.basecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShooterSetHoodPositionCommand extends InstantCommand {
    public ShooterSetHoodPositionCommand(ShooterSubsystem shooterSubsystem, double position) {
        super(() -> shooterSubsystem.setHoodPosition(position));
    }
}

