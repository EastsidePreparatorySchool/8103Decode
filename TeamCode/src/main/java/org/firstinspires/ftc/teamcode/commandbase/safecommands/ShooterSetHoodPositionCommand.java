package org.firstinspires.ftc.teamcode.commandbase.safecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;

public class ShooterSetHoodPositionCommand extends InstantCommand {
    public ShooterSetHoodPositionCommand(HoodSubsystem hoodSubsystem, double position) {
        super(() -> hoodSubsystem.setHoodPosition(position));
    }

    public ShooterSetHoodPositionCommand(double position) {
        this(RobotHardware.getInstance().hoodSubsystem, position);
    }
}
