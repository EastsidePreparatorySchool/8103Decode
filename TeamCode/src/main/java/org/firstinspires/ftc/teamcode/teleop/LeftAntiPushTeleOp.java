package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.complexcommands.AntiPushDriveCommand;

/**
 * Left side TeleOp with full anti-push velocity correction.
 * Uses feedforward + feedback for velocity tracking while driving.
 */
@TeleOp(name = "Left Anti-Push TeleOp", group = "TeleOps")
public class LeftAntiPushTeleOp extends LeftAutoIntakeTeleOp {
    @Override
    protected void initDriveCommand() {
        driveCommand = null;
    }

    @Override
    public void initialize() {
        super.initialize();
        scheduler.setDefaultCommand(
                robot.mecanumSubsystem,
                new AntiPushDriveCommand(gamepad1, true)
        );
    }
}
