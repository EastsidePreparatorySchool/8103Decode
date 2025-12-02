package org.firstinspires.ftc.teamcode.commandbase.safecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;

public class HoodSetPositionCommand extends InstantCommand {
    public HoodSetPositionCommand(HoodSubsystem hoodSubsystem, double position) {
        super(() -> hoodSubsystem.setHoodPosition(position));
        addRequirements(hoodSubsystem);
    }

    public HoodSetPositionCommand(double position) {
        this(RobotHardware.getInstance().hoodSubsystem, position);
    }
}
