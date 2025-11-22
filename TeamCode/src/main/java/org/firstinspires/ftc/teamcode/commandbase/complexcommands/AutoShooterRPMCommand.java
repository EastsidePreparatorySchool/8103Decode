package org.firstinspires.ftc.teamcode.commandbase.complexcommands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
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
        double distance = Math.hypot(Common.ACTUAL_TARGET_X_IN - RobotHardware.getInstance().turretSubsystem.turretX,
                Common.ACTUAL_TARGET_Y_IN - RobotHardware.getInstance().turretSubsystem.turretY);
        double targetRpm = 0;
        try {
            targetRpm = Common.shooterInterpLUT.get(distance);
        } catch (Exception e) {
            targetRpm = 0;
        }

        shooterSubsystem.setTargetRpm(targetRpm);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
