package org.firstinspires.ftc.teamcode.commandbase.safecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

public class PinpointSetPoseCommand extends InstantCommand {
    public PinpointSetPoseCommand(PinpointSubsystem pinpointSubsystem,
                                  double xInches,
                                  double yInches,
                                  double headingDegrees) {
        super(() -> pinpointSubsystem.initializePose(xInches, yInches, headingDegrees));
        addRequirements(pinpointSubsystem);
    }

    public PinpointSetPoseCommand(double xInches,
                                  double yInches,
                                  double headingDegrees) {
        this(RobotHardware.getInstance().pinpointSubsystem, xInches, yInches, headingDegrees);
    }
}
