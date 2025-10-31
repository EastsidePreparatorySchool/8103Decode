package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.basecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

public class PinpointInitializePoseCommand extends InstantCommand {
    public PinpointInitializePoseCommand(PinpointSubsystem pinpointSubsystem,
                                         double xInches,
                                         double yInches,
                                         double headingDegrees) {
        super(() -> {
            if (pinpointSubsystem != null) {
                pinpointSubsystem.initializePose(xInches, yInches, headingDegrees);
            }
        });
    }

    public PinpointInitializePoseCommand(double xInches,
                                         double yInches,
                                         double headingDegrees) {
        this(RobotHardware.getInstance().pinpointSubsystem, xInches, yInches, headingDegrees);
    }
}
