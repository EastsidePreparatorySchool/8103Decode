package org.firstinspires.ftc.teamcode.commandbase.safecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;

public class ColorSensorStateCommand extends InstantCommand {
    public ColorSensorStateCommand(ColorSensorSubsystem.ColorSensorState state) {
        this(RobotHardware.getInstance().colorSensorSubsystem, state);
    }

    public ColorSensorStateCommand(ColorSensorSubsystem colorSensorSubsystem, ColorSensorSubsystem.ColorSensorState state) {
        super(() -> colorSensorSubsystem.setState(state));
        addRequirements(colorSensorSubsystem);
    }
}
