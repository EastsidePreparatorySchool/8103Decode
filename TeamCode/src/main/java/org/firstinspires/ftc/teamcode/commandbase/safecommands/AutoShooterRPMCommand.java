package org.firstinspires.ftc.teamcode.commandbase.safecommands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class AutoShooterRPMCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final PinpointSubsystem pinpointSubsystem;

    public AutoShooterRPMCommand(ShooterSubsystem shooterSubsystem, PinpointSubsystem pinpointSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.pinpointSubsystem = pinpointSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        double currentX = pinpointSubsystem.getXInches();
        double currentY = pinpointSubsystem.getYInches();

        double distance = Math.hypot(Common.SELECTED_FIELD_TARGET_X_IN - currentX,
                Common.SELECTED_FIELD_TARGET_Y_IN - currentY);

        double targetRpm = Common.shooterInterpLUT.get(distance);

        shooterSubsystem.setTargetRpm(targetRpm);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
