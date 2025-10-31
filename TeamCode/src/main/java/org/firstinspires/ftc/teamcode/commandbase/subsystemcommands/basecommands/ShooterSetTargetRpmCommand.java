package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.basecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShooterSetTargetRpmCommand extends InstantCommand {
    public ShooterSetTargetRpmCommand(ShooterSubsystem shooterSubsystem, double targetRpm) {
        super(() -> shooterSubsystem.setTargetRpm(targetRpm));
    }
}

