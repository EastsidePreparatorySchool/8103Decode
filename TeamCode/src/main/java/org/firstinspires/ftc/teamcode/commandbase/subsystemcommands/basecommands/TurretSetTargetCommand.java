package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.basecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretSetTargetCommand extends InstantCommand {
    public TurretSetTargetCommand(TurretSubsystem turretSubsystem, double targetDegrees) {
        super(() -> turretSubsystem.setTarget(targetDegrees));
    }
}
