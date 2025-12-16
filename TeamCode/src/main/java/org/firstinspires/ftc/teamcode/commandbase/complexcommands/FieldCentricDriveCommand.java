package org.firstinspires.ftc.teamcode.commandbase.complexcommands;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

/**
 * Simple field-centric drive command with robot-centric failsafe.
 * 
 * Uses Pinpoint heading to rotate joystick inputs from field frame to robot frame.
 * Falls back to robot-centric if Pinpoint fails or BACK button pressed.
 * 
 * Controls (Gamepad 1):
 * - Left stick: Translation (field-centric, or robot-centric in failsafe)
 * - Triggers: Rotation (right = CW, left = CCW)
 * - BACK: Toggle robot-centric failsafe mode
 */
@Config
public class FieldCentricDriveCommand extends CommandBase {
    
    protected final MecanumSubsystem mecanumSubsystem;
    protected final Gamepad gamepad;
    protected final RobotHardware robot;
    
    /**
     * Side offset for alliance-specific driving.
     * true = left side (offset 0°), false = right side (offset 180°), null = no offset
     */
    protected final Boolean side;
    
    // Tunable multiplier
    public static double DRIVE_MULTIPLIER = 0.85;
    public static double TURN_SENSITIVITY = 1.5;  // Divider for turn (higher = slower)
    
    // Failsafe state
    protected boolean robotCentricMode = false;
    protected boolean prevBackButton = false;
    protected String failsafeReason = "";
    
    public FieldCentricDriveCommand(Gamepad gamepad, Boolean side) {
        this.robot = RobotHardware.getInstance();
        this.mecanumSubsystem = robot.mecanumSubsystem;
        this.gamepad = gamepad;
        this.side = side;
        addRequirements(mecanumSubsystem);
    }
    
    public FieldCentricDriveCommand(Gamepad gamepad) {
        this(gamepad, null);
    }
    
    @Override
    public void execute() {
        // === FAILSAFE TOGGLE (Back button) ===
        boolean backButton = gamepad.back;
        if (backButton && !prevBackButton) {
            robotCentricMode = !robotCentricMode;
            failsafeReason = robotCentricMode ? "Manual toggle" : "";
        }
        prevBackButton = backButton;
        
        // === FAILSAFE CHECK: Pinpoint data validity ===
        double heading = robot.pinpointSubsystem.getHeadingRadians();
        if (Double.isNaN(heading) || Double.isInfinite(heading)) {
            if (!robotCentricMode) {
                robotCentricMode = true;
                failsafeReason = "Pinpoint NaN";
            }
            heading = 0;  // Use 0 for robot-centric
        }
        
        // Read driver inputs
        double forward = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = gamepad.right_trigger - gamepad.left_trigger;
        
        // Cube for finer control at low speeds
        forward = forward * forward * forward;
        strafe = strafe * strafe * strafe;
        turn = (turn * turn * turn) / TURN_SENSITIVITY;
        
        // Field-centric rotation (skip if in robot-centric failsafe mode)
        if (side != null && !robotCentricMode) {
            double offset = side ? 0 : Math.PI;  // Left = 0, Right = 180°
            double angle = -(heading - offset);
            
            double oldForward = forward;
            double oldStrafe = strafe;
            strafe = oldStrafe * Math.cos(angle) - oldForward * Math.sin(angle);
            forward = oldStrafe * Math.sin(angle) + oldForward * Math.cos(angle);
        }
        
        // Mecanum kinematics
        double multiplier = DRIVE_MULTIPLIER;
        double denominator = Math.max(1.0, multiplier * (Math.abs(forward) + Math.abs(strafe) + Math.abs(turn)));
        
        double powerFL = multiplier * (forward + strafe + turn) / denominator;
        double powerFR = multiplier * (forward - strafe - turn) / denominator;
        double powerBL = multiplier * (forward - strafe + turn) / denominator;
        double powerBR = multiplier * (forward + strafe - turn) / denominator;
        
        // Apply to motors
        mecanumSubsystem.setMotorPowers(powerFL, powerFR, powerBL, powerBR);
    }
    
    // Getters for telemetry
    public boolean isRobotCentricMode() { return robotCentricMode; }
    public String getFailsafeReason() { return failsafeReason; }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        mecanumSubsystem.stop();
    }
}
