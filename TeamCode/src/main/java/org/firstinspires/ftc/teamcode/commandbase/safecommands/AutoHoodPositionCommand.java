package org.firstinspires.ftc.teamcode.commandbase.safecommands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

public class AutoHoodPositionCommand extends CommandBase {
    private final HoodSubsystem hoodSubsystem;
    private final PinpointSubsystem pinpointSubsystem;

    public AutoHoodPositionCommand(HoodSubsystem hoodSubsystem, PinpointSubsystem pinpointSubsystem) {
        this.hoodSubsystem = hoodSubsystem;
        this.pinpointSubsystem = pinpointSubsystem;
        addRequirements(hoodSubsystem);
    }

    @Override
    public void execute() {
        double currentX = pinpointSubsystem.getXInches();
        double currentY = pinpointSubsystem.getYInches();

        double distance = Math.hypot(Common.SELECTED_FIELD_TARGET_X_IN - currentX,
                Common.SELECTED_FIELD_TARGET_Y_IN - currentY);

        double targetHoodPos = Common.hoodInterpLUT.get(distance);

        hoodSubsystem.setPosition(targetHoodPos);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
