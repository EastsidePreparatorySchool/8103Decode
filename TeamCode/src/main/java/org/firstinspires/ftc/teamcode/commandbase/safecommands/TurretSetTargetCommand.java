package org.firstinspires.ftc.teamcode.commandbase.unsafebasecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretSetTargetCommand extends InstantCommand {
    public TurretSetTargetCommand(TurretSubsystem turretSubsystem, double targetDegrees) {
        super(() -> turretSubsystem.setTarget(targetDegrees));
    }

    public TurretSetTargetCommand(double targetDegrees) {
        this(RobotHardware.getInstance().turretSubsystem, targetDegrees);
    }
}
