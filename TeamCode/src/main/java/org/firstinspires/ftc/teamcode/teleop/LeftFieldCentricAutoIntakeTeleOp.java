package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.complexcommands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.lib.Common;

/**
 * Field-centric TeleOp for the LEFT side of the field.
 * Joystick inputs are relative to the field rather than the robot's orientation.
 * The robot will move in the direction the joystick is pushed regardless of which way the robot is facing.
 */
@TeleOp(name = "LeftFieldCentricAutoIntakeTeleOp", group = "TeleOps")
public class LeftFieldCentricAutoIntakeTeleOp extends AutoIntakeTeleOp {
    @Override
    public void initialize() {
        Common.TARGET_X_IN = Common.LEFT_FIELD_TARGET_X_IN;
        Common.TARGET_Y_IN = Common.LEFT_FIELD_TARGET_Y_IN;
        Common.START_X_IN = Common.LEFT_FAR_START_X_IN;
        Common.START_Y_IN = Common.LEFT_FAR_START_Y_IN;
        Common.START_HEADING_DEG = Common.LEFT_FAR_START_HEADING_DEG;
        Common.ACTUAL_TARGET_X_IN = Common.LEFT_FIELD_ACTUAL_TARGET_X_IN;
        Common.ACTUAL_TARGET_Y_IN = Common.LEFT_FIELD_ACTUAL_TARGET_Y_IN;
        super.initialize();
    }

    @Override
    protected void initDriveCommand() {
        // Left side: driver's forward = field 0° direction
        // Robot starts at 90° but joystick forward should move robot toward field 0°
        driveCommand = new DriveWithGamepadCommand(gamepad1, 0, true);
    }
}
