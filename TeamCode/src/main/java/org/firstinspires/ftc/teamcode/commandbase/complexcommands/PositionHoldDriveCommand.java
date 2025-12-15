package org.firstinspires.ftc.teamcode.commandbase.complexcommands;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commandbase.safecommands.MecanumPowerMotorsCommand;
import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

/**
 * Drive command with position hold when idle.
 * Normal driving when joystick active, holds position when idle + robot stopped.
 * Uses radians internally, field-centric error, proper vector clamping.
 */
@Config
public class PositionHoldDriveCommand extends CommandBase {
    protected final MecanumSubsystem mecanumSubsystem;
    protected final Gamepad gamepad;
    protected final RobotHardware robot;
    protected final Boolean side;

    // === TUNABLE CONSTANTS ===
    // === TUNABLE CONSTANTS ===
    public static double HOLD_KP = 0.08;           // power per inch error
    public static double HOLD_KD = 0.015;          // power per in/s velocity (damping)
    public static double HOLD_H_KP = 0.8;          // power per radian heading error
    public static double HOLD_H_KD = 0.1;          // power per rad/s heading velocity
    
    public static double MAX_HOLD_XY_POWER = 0.4;  // max correction power (translation)
    public static double MAX_HOLD_H_POWER = 0.3;   // max correction power (heading)
    
    public static double POSITION_DEADBAND_ENTER = 0.3;  // inches
    public static double POSITION_DEADBAND_EXIT = 0.6;   // inches
    public static double HEADING_DEADBAND = 0.035;       // radians (~2 degrees)
    
    public static double INPUT_DEADBAND_ENTER = 0.05;
    public static double INPUT_DEADBAND_EXIT = 0.10;
    
    public static double VELOCITY_THRESHOLD = 3.0; // in/s - must be below to enter hold
    public static double OMEGA_THRESHOLD = 0.5;    // rad/s - must be below to enter hold
    
    public static double SLEW_RATE_PER_SEC = 2.5;    // max power change per second (translation)
    public static double SLEW_RATE_H_PER_SEC = 3.0;  // max power change per second (heading)
    
    public static double FILTER_TAU = 0.08;  // velocity filter time constant (seconds)

    /**
     * === AXIS CONVENTION ===
     * 
     * POSITION: Pinpoint reports FIELD-centric position (X, Y in field coordinates).
     * 
     * VELOCITY: Pinpoint reports ROBOT-centric velocity:
     *   - velX: positive = robot moving FORWARD (relative to robot front)
     *   - velY: positive = robot moving LEFT (relative to robot)
     *   - velH: positive = counterclockwise (CCW) rotation
     * 
     * Mecanum mixing convention:
     *   - forward: positive = forward motion
     *   - strafe: positive = left motion  
     *   - turn: positive = CCW rotation
     * 
     * VERIFICATION: Run PositionHoldTuning, push robot forward (while robot faces any direction),
     * check that Vel/X is positive. Push robot left (sideways), check Vel/Y is positive.
     * This should be true regardless of robot heading since velocity is ROBOT-centric.
     */

    // State
    protected boolean isHolding = false;
    protected double holdX, holdY, holdHeadingRad;
    protected double lastCorrX = 0, lastCorrY = 0, lastCorrH = 0;
    protected boolean positionInDeadband = false;
    protected boolean positionInDeadband = false;
    protected long lastTimeNs = System.nanoTime();
    
    // Hold Entry Timer
    protected long holdEntryTimerNs = 0;
    protected boolean pendingHold = false;
    public static double HOLD_CAPTURE_DELAY_MS = 100; // ms to wait below threshold before holding

    // Diagnostics / Telemetry Data
    protected double diagFiltVelX, diagFiltVelY, diagFiltVelH; // Filtered velocities
    protected double diagErrFieldX, diagErrFieldY, diagErrH;   // Errors
    protected double diagCorrFieldX, diagCorrFieldY;           // Field frame corrections
    protected double diagCorrRobotX, diagCorrRobotY, diagCorrH; // Final robot frame corrections
    protected boolean diagXyClamped = false;
    protected boolean diagHClamped = false;
    protected boolean diagSlewXyActive = false;
    protected boolean diagSlewHActive = false;
    protected double diagAlpha = 0; // Filter alpha
    protected double diagDt = 0;    // Delta time

    // Velocity filter (low-pass)
    protected double filtVelX = 0, filtVelY = 0, filtVelH = 0;

    public PositionHoldDriveCommand(Gamepad gamepad, Boolean side) {
        this.robot = RobotHardware.getInstance();
        this.mecanumSubsystem = robot.mecanumSubsystem;
        this.gamepad = gamepad;
        this.side = side;
        addRequirements(mecanumSubsystem);
    }

    public PositionHoldDriveCommand(Gamepad gamepad) {
        this(gamepad, null);
    }

    @Override
    public void execute() {
        // Time delta with clamp
        long nowNs = System.nanoTime();
        double dt = (nowNs - lastTimeNs) / 1e9;
        lastTimeNs = nowNs;
        dt = Math.max(0.005, Math.min(dt, 0.05));

        // Get filtered velocities (field-centric from Pinpoint)
        double rawVelX = robot.pinpointSubsystem.getVelXInPerSec();
        double rawVelY = robot.pinpointSubsystem.getVelYInPerSec();
        double rawVelH = robot.pinpointSubsystem.getVelHeadingRadPerSec();
        
        // Time-constant based filter alpha
        double alpha = dt / (FILTER_TAU + dt);
        
        filtVelX += alpha * (rawVelX - filtVelX);
        filtVelY += alpha * (rawVelY - filtVelY);
        filtVelH += alpha * (rawVelH - filtVelH);

        // Update diagnostics
        diagFiltVelX = filtVelX;
        diagFiltVelY = filtVelY;
        diagFiltVelH = filtVelH;
        diagAlpha = alpha;
        diagDt = dt;

        // Read gamepad
        double forward = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = gamepad.right_trigger - gamepad.left_trigger;

        // Cube for finer control
        forward = Math.pow(forward, 3);
        strafe = Math.pow(strafe, 3);
        turn = Math.pow(turn, 3) / 1.5;

        double inputMag = Math.hypot(forward, strafe) + Math.abs(turn);
        double velMag = Math.hypot(filtVelX, filtVelY);
        double omegaMag = Math.abs(filtVelH);

        // Hysteresis idle detection
        if (isHolding && inputMag > INPUT_DEADBAND_EXIT) {
            isHolding = false;
            pendingHold = false;
        }
        
        // Hold Entry Logic with Time Gate
        if (!isHolding) {
            boolean conditionsMet = inputMag < INPUT_DEADBAND_ENTER 
                    && velMag < VELOCITY_THRESHOLD
                    && omegaMag < OMEGA_THRESHOLD;
            
            if (conditionsMet) {
                if (!pendingHold) {
                    // Start timer
                    pendingHold = true;
                    holdEntryTimerNs = nowNs;
                } else {
                    // Check timer
                    double elapsedMs = (nowNs - holdEntryTimerNs) / 1e6;
                    if (elapsedMs > HOLD_CAPTURE_DELAY_MS) {
                        isHolding = true;
                        pendingHold = false;
                        
                        // Capture hold pose
                        holdX = robot.pinpointSubsystem.getXInches();
                        holdY = robot.pinpointSubsystem.getYInches();
                        holdHeadingRad = robot.pinpointSubsystem.getHeadingRadians();
                        lastCorrX = 0; lastCorrY = 0; lastCorrH = 0;
                        positionInDeadband = false;
                    }
                }
            } else {
                // Reset timer if conditions lost
                pendingHold = false;
            }
        }

        double powerFL, powerFR, powerBL, powerBR;

        if (isHolding) {
            // Position hold mode - PD controller
            double currentX = robot.pinpointSubsystem.getXInches();
            double currentY = robot.pinpointSubsystem.getYInches();
            double currentH = robot.pinpointSubsystem.getHeadingRadians();

            // Error in field frame
            double errFieldX = holdX - currentX;
            double errFieldY = holdY - currentY;
            double errH = normalizeAngle(holdHeadingRad - currentH);

            // Apply hysteresis deadband (in field frame)
            double posErrorMag = Math.hypot(errFieldX, errFieldY);
            if (positionInDeadband && posErrorMag > POSITION_DEADBAND_EXIT) {
                positionInDeadband = false;
            }
            if (!positionInDeadband && posErrorMag < POSITION_DEADBAND_ENTER) {
                positionInDeadband = true;
            }
            if (positionInDeadband) {
                errFieldX = 0;
                errFieldY = 0;
            }

            if (Math.abs(errH) < HEADING_DEADBAND) errH = 0;

            // Rotate field error to ROBOT frame (since velocity is robot-centric)
            // This way both P and D terms are in the same frame
            double offset = (side != null && !side) ? Math.PI : 0.0;
            double angle = -(currentH - offset);
            
            double errRobotX = errFieldX * Math.cos(angle) - errFieldY * Math.sin(angle);
            double errRobotY = errFieldX * Math.sin(angle) + errFieldY * Math.cos(angle);

            // PD in ROBOT frame (velocity filtVelX/Y is already robot-centric)
            // P term: push toward target
            // D term: damp velocity (both now in robot frame)
            double corrRobotX = errRobotX * HOLD_KP - filtVelX * HOLD_KD;
            double corrRobotY = errRobotY * HOLD_KP - filtVelY * HOLD_KD;
            double corrH = errH * HOLD_H_KP - filtVelH * HOLD_H_KD;
            
            // Diagnostics
            diagErrFieldX = errFieldX;
            diagErrFieldY = errFieldY;
            diagErrH = errH;
            diagCorrFieldX = corrRobotX; // Technically "uncapped robot correction" at this stage but mapped to field semantics
            diagCorrFieldY = corrRobotY; 

            // Separate XY and Heading power caps
            double corrMag = Math.hypot(corrRobotX, corrRobotY);
            
            diagXyClamped = false;
            if (corrMag > MAX_HOLD_XY_POWER) {
                double scale = MAX_HOLD_XY_POWER / corrMag;
                corrFieldX *= scale;
                corrFieldY *= scale;
                diagXyClamped = true;
            }
            // Apply consistent side offset for hold mode (same as driving)
            // ... (rotation logic) ...
            
            offset = (side != null && !side) ? Math.PI : 0.0;
            angle = -(currentH - offset);
            
            double corrRobotX = corrFieldX * Math.cos(angle) - corrFieldY * Math.sin(angle);
            double corrRobotY = corrFieldX * Math.sin(angle) + corrFieldY * Math.cos(angle);
            
            // Diagnostics: Robot frame corrections before slew
            diagCorrRobotX = corrRobotX;
            diagCorrRobotY = corrRobotY;

            diagHClamped = false;
            double oldCorrH = corrH;
            corrH = Math.max(-MAX_HOLD_H_POWER, Math.min(MAX_HOLD_H_POWER, corrH));
            if (Math.abs(corrH) < Math.abs(oldCorrH)) diagHClamped = true;

            // Slew rate limit on delta vector (XY)
            double deltaX = corrRobotX - lastCorrX;
            double deltaY = corrRobotY - lastCorrY;
            double deltaMag = Math.hypot(deltaX, deltaY);
            double maxDelta = SLEW_RATE_PER_SEC * dt;
            
            diagSlewXyActive = false;
            if (deltaMag > maxDelta && deltaMag > 0.001) {
                double scale = maxDelta / deltaMag;
                deltaX *= scale;
                deltaY *= scale;
                diagSlewXyActive = true;
            }
            corrRobotX = lastCorrX + deltaX;
            corrRobotY = lastCorrY + deltaY;
            lastCorrX = corrRobotX;
            lastCorrY = corrRobotY;
            
            // Slew rate limit on Heading
            double deltaH = corrH - lastCorrH;
            double maxDeltaH = SLEW_RATE_H_PER_SEC * dt;
            
            diagSlewHActive = false;
            if (Math.abs(deltaH) > maxDeltaH) {
                deltaH = Math.signum(deltaH) * maxDeltaH;
                diagSlewHActive = true;
            }
            corrH = lastCorrH + deltaH;
            lastCorrH = corrH;
            
            // Final Diagnostics
            diagCorrH = corrH;

            // EXPLICIT AXIS MAPPING
            // Corrections are now in robot frame
            double corrForward = corrRobotX;  // X is Forward
            double corrStrafe = corrRobotY;   // Y is Strafe (Left)

            // Mecanum kinematics
            powerFL = corrForward + corrStrafe + corrH;
            powerFR = corrForward - corrStrafe - corrH;
            powerBL = corrForward - corrStrafe + corrH;
            powerBR = corrForward + corrStrafe - corrH;

            // Normalize if needed
            double maxPower = Math.max(1.0, Math.max(
                    Math.max(Math.abs(powerFL), Math.abs(powerFR)),
                    Math.max(Math.abs(powerBL), Math.abs(powerBR))));
            powerFL /= maxPower;
            powerFR /= maxPower;
            powerBL /= maxPower;
            powerBR /= maxPower;

        } else {
            // Normal driving - use overridable method
            double[] powers = computeDrivePowers(forward, strafe, turn, dt);
            powerFL = powers[0];
            powerFR = powers[1];
            powerBL = powers[2];
            powerBR = powers[3];
            
            // Clear hold diagnostics
            diagErrFieldX = 0; diagErrFieldY = 0; diagErrH = 0;
            diagCorrFieldX = 0; diagCorrFieldY = 0;
            diagXyClamped = false; diagHClamped = false;
            diagSlewXyActive = false; diagSlewHActive = false;
        }

        // Direct motor control (Fix 1: No scheduler spam)
        mecanumSubsystem.setMotorPowers(powerFL, powerFR, powerBL, powerBR);
    }

    public boolean isHolding() {
        return isHolding;
    }

    public double getHoldX() { return holdX; }
    public double getHoldY() { return holdY; }
    public double getHoldHeadingRad() { return holdHeadingRad; }

    // === DIAGNOSTIC GETTERS ===
    public double getFiltVelX() { return diagFiltVelX; }
    public double getFiltVelY() { return diagFiltVelY; }
    public double getFiltVelH() { return diagFiltVelH; }
    public double getErrFieldX() { return diagErrFieldX; }
    public double getErrFieldY() { return diagErrFieldY; }
    public double getErrH() { return diagErrH; }
    public double getCorrFieldX() { return diagCorrFieldX; }
    public double getCorrFieldY() { return diagCorrFieldY; }
    public double getCorrRobotX() { return diagCorrRobotX; }
    public double getCorrRobotY() { return diagCorrRobotY; }
    public double getCorrH() { return diagCorrH; }
    public boolean isXyClamped() { return diagXyClamped; }
    public boolean isHClamped() { return diagHClamped; }
    public boolean isSlewXyActive() { return diagSlewXyActive; }
    public boolean isSlewHActive() { return diagSlewHActive; }
    public boolean isPositionInDeadband() { return positionInDeadband; }
    public double getFilterAlpha() { return diagAlpha; }
    public double getDt() { return diagDt; }

    protected double normalizeAngle(double angle) {
        // Fix 8: Safe normalization
        if (Double.isNaN(angle) || Double.isInfinite(angle)) return 0;
        return Math.IEEEremainder(angle, 2 * Math.PI);
    }

    /**
     * Compute drive motor powers when NOT holding position.
     * Override this in subclasses to add velocity correction.
     */
    protected double[] computeDrivePowers(double forward, double strafe, double turn, double dt) {
        if (side != null) {
            double heading = robot.pinpointSubsystem.getHeadingRadians();
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

        lastCorrX = 0;
        lastCorrY = 0;

        return new double[]{powerFL, powerFR, powerBL, powerBR};
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
