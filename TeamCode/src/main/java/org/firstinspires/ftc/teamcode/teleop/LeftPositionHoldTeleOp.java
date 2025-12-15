package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.complexcommands.PositionHoldDriveCommand;

/**
 * Left side TeleOp with position hold when idle.
 * Uses PositionHoldDriveCommand instead of standard DriveWithGamepadCommand.
 */
@TeleOp(name = "Left Position Hold TeleOp", group = "TeleOps")
public class LeftPositionHoldTeleOp extends LeftAutoIntakeTeleOp {
    @Override
    protected void initDriveCommand() {
        // Use position hold drive command instead of standard
        driveCommand = null;  // Don't use the standard one
    }

    @Override
    public void initialize() {
        super.initialize();
        // Override the default command with position hold version
        scheduler.setDefaultCommand(
                robot.mecanumSubsystem,
                new PositionHoldDriveCommand(gamepad1, true)  // true = left side
        );
    }
}
