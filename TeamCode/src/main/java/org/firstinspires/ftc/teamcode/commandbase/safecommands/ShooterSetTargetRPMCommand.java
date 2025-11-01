package org.firstinspires.ftc.teamcode.commandbase.safecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShooterSetTargetRPMCommand extends InstantCommand {
    public ShooterSetTargetRPMCommand(ShooterSubsystem shooterSubsystem, double targetRpm) {
        super(() -> shooterSubsystem.setTargetRpm(targetRpm));
    }
}

