package org.firstinspires.ftc.teamcode.commandbase.unsafebasecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;

public class HoodSetPositionCommand extends InstantCommand {
    public HoodSetPositionCommand(HoodSubsystem hoodSubsystem, double position) {
        super(() -> hoodSubsystem.setHoodPosition(position));
    }

    public HoodSetPositionCommand(double position) {
        this(RobotHardware.getInstance().hoodSubsystem, position);
    }
}
