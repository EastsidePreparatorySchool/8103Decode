package org.firstinspires.ftc.teamcode.commandbase.safecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;

public class DistanceSensorStateCommand extends InstantCommand {
    public DistanceSensorStateCommand(DistanceSensorSubsystem distanceSensorSubsystem, DistanceSensorSubsystem.DistanceSensorState state) {
        super(() -> distanceSensorSubsystem.setState(state));
        addRequirements(distanceSensorSubsystem);
    }

    public DistanceSensorStateCommand(DistanceSensorSubsystem.DistanceSensorState state) {
        this(RobotHardware.getInstance().distanceSensorSubsystem, state);
    }
}
