package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.complexcommands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.lib.Common;

/**
 * Field-centric TeleOp for the RIGHT side of the field.
 * Joystick inputs are relative to the field rather than the robot's orientation.
 * The robot will move in the direction the joystick is pushed regardless of which way the robot is facing.
 */
@TeleOp(name = "RightFieldCentricAutoIntakeTeleOp", group = "TeleOps")
public class RightFieldCentricAutoIntakeTeleOp extends AutoIntakeTeleOp {
    @Override
    public void initialize() {
        Common.TARGET_X_IN = Common.RIGHT_FIELD_TARGET_X_IN;
        Common.TARGET_Y_IN = Common.RIGHT_FIELD_TARGET_Y_IN;
        Common.START_X_IN = Common.RIGHT_FAR_START_X_IN;
        Common.START_Y_IN = Common.RIGHT_FAR_START_Y_IN;
        Common.ACTUAL_TARGET_X_IN = Common.RIGHT_FIELD_ACTUAL_TARGET_X_IN;
        Common.ACTUAL_TARGET_Y_IN = Common.RIGHT_FIELD_ACTUAL_TARGET_Y_IN;
        super.initialize();
    }

    @Override
    protected void initDriveCommand() {
        // Right side: forward on joystick = right on field (+90° = PI/2 radians)
        // Use right stick X for turning instead of triggers
        driveCommand = new DriveWithGamepadCommand(gamepad1, Math.PI, true);
    }
}
