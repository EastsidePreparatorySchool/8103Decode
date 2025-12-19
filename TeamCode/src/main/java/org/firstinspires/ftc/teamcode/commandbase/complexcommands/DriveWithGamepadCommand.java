package org.firstinspires.ftc.teamcode.commandbase.complexcommands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commandbase.safecommands.MecanumPowerMotorsCommand;
import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;

public class DriveWithGamepadCommand extends CommandBase {
    private final MecanumSubsystem mecanumSubsystem;
    private final Gamepad gamepad;
    private final Boolean side;
    private final Double angleOffsetRadians; // Custom angle offset for field-centric driving
    private final boolean swapSticks; // If true, right stick = drive/strafe, left stick X = turn

    public DriveWithGamepadCommand(MecanumSubsystem mecanumSubsystem, Gamepad gamepad, Boolean side, Double angleOffsetRadians, boolean useRightStickTurn) {
        this.mecanumSubsystem = mecanumSubsystem;
        this.gamepad = gamepad;
        this.side = side;
        this.angleOffsetRadians = angleOffsetRadians;
        this.swapSticks = useRightStickTurn;
        addRequirements(mecanumSubsystem);
    }

    public DriveWithGamepadCommand(MecanumSubsystem mecanumSubsystem, Gamepad gamepad, Boolean side, Double angleOffsetRadians) {
        this(mecanumSubsystem, gamepad, side, angleOffsetRadians, false);
    }

    public DriveWithGamepadCommand(MecanumSubsystem mecanumSubsystem, Gamepad gamepad, Boolean side) {
        this(mecanumSubsystem, gamepad, side, null);
    }

    public DriveWithGamepadCommand(MecanumSubsystem mecanumSubsystem, Gamepad gamepad) {
        this(mecanumSubsystem, gamepad, null);
    }

    public DriveWithGamepadCommand(Gamepad gamepad) {
        this(
                RobotHardware.getInstance().mecanumSubsystem,
                gamepad,
                null
        );
    }

    public DriveWithGamepadCommand(Gamepad gamepad, Boolean side) {
        this(
                RobotHardware.getInstance().mecanumSubsystem,
                gamepad,
                side
        );
    }

    /**
     * Constructor with custom angle offset for rotated field-centric driving.
     * @param gamepad The gamepad to read inputs from
     * @param angleOffsetRadians The angle offset in radians (e.g., PI/2 for 90° rotation)
     */
    public DriveWithGamepadCommand(Gamepad gamepad, double angleOffsetRadians) {
        this(
                RobotHardware.getInstance().mecanumSubsystem,
                gamepad,
                null,
                angleOffsetRadians,
                false
        );
    }

    /**
     * Constructor with custom angle offset and swapped sticks for field-centric driving.
     * @param gamepad The gamepad to read inputs from
     * @param angleOffsetRadians The angle offset in radians (e.g., PI/2 for 90° rotation)
     * @param swapSticks If true, right stick = drive/strafe, left stick X = turn
     */
    public DriveWithGamepadCommand(Gamepad gamepad, double angleOffsetRadians, boolean swapSticks) {
        this(
                RobotHardware.getInstance().mecanumSubsystem,
                gamepad,
                null,
                angleOffsetRadians,
                swapSticks
        );
    }

    @Override
    public void execute() {
        double forward, strafe, turn;
        
        if (swapSticks) {
            // Swapped: right stick for drive/strafe, left stick for turn (angle-based)
            forward = -gamepad.right_stick_y;
            strafe = gamepad.right_stick_x;
            // Use angle of left stick for yaw - this way 45° still gives full turn
            // Magnitude determines power, X direction determines left/right
            double leftMagnitude = Math.hypot(gamepad.left_stick_x, gamepad.left_stick_y);
            turn = leftMagnitude * Math.signum(gamepad.left_stick_x);
        } else {
            // Default: left stick for drive/strafe, triggers for turn
            forward = -gamepad.left_stick_y;
            strafe = gamepad.left_stick_x;
            turn = gamepad.right_trigger - gamepad.left_trigger;
        }

        // Cubed control for finer low-end response
        forward = Math.pow(forward, 3);
        strafe = Math.pow(strafe, 3);
        turn = Math.pow(turn, 3) / 1.5; // reduce turning sensitivity a bit

        // Apply field-centric transformation if custom angle offset is provided
        // This ONLY rotates joystick inputs - does not affect robot internal coordinates
        // Using standard gm0 field-centric formula
        if (angleOffsetRadians != null) {
            // Get heading from pinpoint (already set in PinpointSubsystem.periodic())
            double botHeading = Math.toRadians(RobotHardware.getInstance().robotHeadingDeg) - angleOffsetRadians;
            
            // Rotate the joystick inputs by the negative of bot heading
            double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);
            
            strafe = rotX;
            forward = rotY;
        } else if (side != null) {
            // Legacy field-centric mode using side boolean
            double heading = Math.toRadians(RobotHardware.getInstance().robotHeadingDeg);
            double offset = side ? 0 : Math.PI;
            double angle = -(heading - offset);
            double oldForward = forward;
            double oldStrafe = strafe;
            strafe = oldStrafe * Math.cos(angle) - oldForward * Math.sin(angle);
            forward = oldStrafe * Math.sin(angle) + oldForward * Math.cos(angle);
        }
        double multiplier = Common.DRIVE_DEFAULT_MULT;
        double denominator = Math.max(multiplier * (Math.abs(forward) + Math.abs(strafe) + Math.abs(turn)), 1.0);
        double powerFL = multiplier * (forward + strafe + turn) / denominator;
        double powerFR = multiplier * (forward - strafe - turn) / denominator;
        double powerBL = multiplier * (forward - strafe + turn) / denominator;
        double powerBR = multiplier * (forward + strafe - turn) / denominator;
        CommandScheduler.getInstance().schedule(
                new MecanumPowerMotorsCommand(
                        mecanumSubsystem,
                        powerFL,
                        powerFR,
                        powerBL,
                        powerBR
                )
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        mecanumSubsystem.stop();
    }
}
