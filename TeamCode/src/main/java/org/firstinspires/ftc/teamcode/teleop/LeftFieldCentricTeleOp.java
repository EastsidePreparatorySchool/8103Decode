package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.complexcommands.FieldCentricDriveCommand;

/**
 * Left side TeleOp with simple field-centric drive.
 * 
 * Inherits from LeftAutoIntakeTeleOp:
 * - Auto spindexer slot switching (ball detection)
 * - Auto shooter RPM and hood position
 * - Turret auto-aiming at target
 * - All gamepad controls (intake, tripleshot, shooter, turret offset)
 * 
 * Drive: Simple field-centric using Pinpoint heading.
 * No position hold or anti-push.
 */
@TeleOp(name = "Left Field-Centric TeleOp", group = "TeleOps")
public class LeftFieldCentricTeleOp extends LeftAutoIntakeTeleOp {
    
    @Override
    protected void initDriveCommand() {
        driveCommand = null;  // Will be set in initialize()
    }
    
    @Override
    public void initialize() {
        super.initialize();
        scheduler.setDefaultCommand(
                robot.mecanumSubsystem,
                new FieldCentricDriveCommand(gamepad1, true)  // true = left side
        );
    }
}
