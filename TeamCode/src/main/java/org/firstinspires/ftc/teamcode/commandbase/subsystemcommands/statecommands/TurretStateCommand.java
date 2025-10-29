package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.statecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretStateCommand extends InstantCommand {
    public TurretStateCommand(TurretSubsystem.TurretState turretState) {
        super(
                () -> RobotHardware.getInstance().turretSubsystem.setTurretState(turretState)
        );
    }
}
