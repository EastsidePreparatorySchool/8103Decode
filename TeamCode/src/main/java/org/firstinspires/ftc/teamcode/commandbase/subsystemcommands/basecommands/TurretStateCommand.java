package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.basecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretStateCommand extends InstantCommand {
    public TurretStateCommand(TurretSubsystem turretSubsystem, TurretSubsystem.TurretState turretState) {
        super(() -> turretSubsystem.setTurretState(turretState));
    }

    public TurretStateCommand(TurretSubsystem.TurretState turretState) {
        this(RobotHardware.getInstance().turretSubsystem, turretState);
    }
}
