package org.firstinspires.ftc.teamcode.commandbase.complexcommands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
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
        double distance = Math.hypot(Common.ACTUAL_TARGET_X_IN - RobotHardware.getInstance().turretSubsystem.turretX,
                Common.ACTUAL_TARGET_Y_IN - RobotHardware.getInstance().turretSubsystem.turretY);

        double targetHoodPos = Common.HOOD_INITIAL_POS;
        try {
            targetHoodPos = Common.hoodInterpLUT.get(distance);
        } catch (Exception e) {
            targetHoodPos = Common.HOOD_INITIAL_POS;
        }

        hoodSubsystem.setHoodPosition(targetHoodPos);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
