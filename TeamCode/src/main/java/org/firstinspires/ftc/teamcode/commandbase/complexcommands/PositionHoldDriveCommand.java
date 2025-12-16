package org.firstinspires.ftc.teamcode.commandbase.complexcommands;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

/**
 * Drive command with position hold when idle.
 *
 * Assumptions:
 * - Pinpoint pose (X,Y,Heading) is FIELD-centric.
 * - Pinpoint velocity (VelX,VelY) is ROBOT-centric (forward, left).
 * - Vel units are inches/sec; heading velocity is rad/sec.
 *
 * Hold control:
 * - Compute FIELD position error to hold pose
 * - Rotate FIELD error -> ROBOT frame (matching your existing field-centric drive rotation convention)
 * - PD in ROBOT frame using ROBOT-centric velocity
 * - Vector clamp, slew-rate limit, mecanum mix
 */
@Config
public class PositionHoldDriveCommand extends CommandBase {

    protected final MecanumSubsystem mecanumSubsystem;
    protected final Gamepad gamepad;
    protected final RobotHardware robot;
    protected final Boolean side;

    // ============================================================================
    // TUNING GUIDE
    // ============================================================================
    // 🔴 MUST TUNE: HOLD_KP, HOLD_KD, HOLD_H_KP, VELOCITY_THRESHOLD
    // 🟡 SET ONCE:  HOLD_H_KD, MAX_HOLD_*_POWER, FILTER_TAU, HOLD_CAPTURE_DELAY_MS
    // 🟢 RARELY TOUCH: SLEW_RATE_*, DEADBAND_*, OMEGA_THRESHOLD
    // ============================================================================
    
    // --- 🔴 MUST TUNE: Core PD gains ---
    public static double HOLD_KP = 0.08;    // Increase until robot resists pushing firmly
    public static double HOLD_KD = 0.015;   // Increase if robot overshoots when released
    public static double HOLD_H_KP = 0.8;   // Increase until robot resists rotation
    public static double VELOCITY_THRESHOLD = 3.0; // Lower if hold triggers while coasting

    // --- 🟡 SET ONCE: Usually fine at defaults ---
    public static double HOLD_H_KD = 0.1;           // Heading damping
    public static double MAX_HOLD_XY_POWER = 0.4;   // Max translation correction (higher for contact games)
    public static double MAX_HOLD_H_POWER  = 0.3;   // Max heading correction
    public static double FILTER_TAU = 0.08;         // Velocity filter time constant (increase if jittery)
    public static double HOLD_CAPTURE_DELAY_MS = 100; // ms below threshold before capturing pose
    public static double OMEGA_THRESHOLD = 0.5;     // rad/s threshold for rotation

    // --- 🟢 RARELY TOUCH: Only if specific issues ---
    public static double SLEW_RATE_PER_SEC   = 2.5;  // Increase if response feels laggy
    public static double SLEW_RATE_H_PER_SEC = 3.0;  // Heading slew rate
    public static double POSITION_DEADBAND_ENTER = 0.3;  // Increase if motors hum at rest
    public static double POSITION_DEADBAND_EXIT  = 0.6;  // Should be > ENTER for hysteresis
    public static double HEADING_DEADBAND = 0.035;       // ~2 degrees
    public static double INPUT_DEADBAND_ENTER = 0.05;    // Joystick threshold to enter hold
    public static double INPUT_DEADBAND_EXIT  = 0.10;    // Joystick threshold to exit hold

    // --- FALLBACK SAFETY ---
    public static double PINPOINT_STALE_THRESHOLD_MS = 500; // If no update for this long, fallback

    // === State ===
    protected boolean fallbackMode = false;  // True = basic drive, no position hold
    protected long lastValidPinpointNs = System.nanoTime();
    protected boolean prevBackButton = false;
    protected String fallbackReason = "";
    protected boolean isHolding = false;
    protected boolean pendingHold = false;
    protected long holdEntryTimerNs = 0;

    protected double holdX, holdY, holdHeadingRad;

    protected double lastCorrX = 0, lastCorrY = 0, lastCorrH = 0;
    protected boolean positionInDeadband = false;

    protected long lastTimeNs = System.nanoTime();

    // Velocity filter state (robot-centric for X/Y, rad/s for H)
    protected double filtVelX = 0, filtVelY = 0, filtVelH = 0;

    // === Diagnostics ===
    protected double diagFiltVelX, diagFiltVelY, diagFiltVelH;
    protected double diagErrFieldX, diagErrFieldY, diagErrH;
    protected double diagErrRobotX, diagErrRobotY;
    protected double diagCorrRobotX, diagCorrRobotY, diagCorrH;
    protected boolean diagXyClamped = false;
    protected boolean diagHClamped  = false;
    protected boolean diagSlewXyActive = false;
    protected boolean diagSlewHActive  = false;
    protected double diagAlpha = 0;
    protected double diagDt = 0;

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
        // dt with clamp
        long nowNs = System.nanoTime();
        double dt = (nowNs - lastTimeNs) / 1e9;
        lastTimeNs = nowNs;
        dt = Math.max(0.005, Math.min(dt, 0.05));
        diagDt = dt;

        // === FALLBACK TOGGLE (Back button) ===
        boolean backButton = gamepad.back;
        if (backButton && !prevBackButton) {
            fallbackMode = !fallbackMode;
            fallbackReason = fallbackMode ? "Manual toggle" : "";
            isHolding = false;
            pendingHold = false;
        }
        prevBackButton = backButton;

        // === FALLBACK CHECK: Pinpoint data validity ===
        double rawVelX = robot.pinpointSubsystem.getVelXInPerSec();
        double rawVelY = robot.pinpointSubsystem.getVelYInPerSec();
        double rawVelH = robot.pinpointSubsystem.getVelHeadingRadPerSec();
        double poseX = robot.pinpointSubsystem.getXInches();
        double poseY = robot.pinpointSubsystem.getYInches();
        double poseH = robot.pinpointSubsystem.getHeadingRadians();

        // Check for NaN or Inf (sensor failure)
        boolean dataInvalid = Double.isNaN(rawVelX) || Double.isNaN(rawVelY) || Double.isNaN(rawVelH)
                || Double.isNaN(poseX) || Double.isNaN(poseY) || Double.isNaN(poseH)
                || Double.isInfinite(rawVelX) || Double.isInfinite(rawVelY) || Double.isInfinite(poseH);

        if (dataInvalid) {
            if (!fallbackMode) {
                fallbackMode = true;
                fallbackReason = "Pinpoint NaN/Inf";
                isHolding = false;
                pendingHold = false;
            }
        } else {
            lastValidPinpointNs = nowNs;
        }

        // Check for stale data (no valid update in threshold time)
        double staleDurationMs = (nowNs - lastValidPinpointNs) / 1e6;
        if (staleDurationMs > PINPOINT_STALE_THRESHOLD_MS && !fallbackMode) {
            fallbackMode = true;
            fallbackReason = "Pinpoint stale (" + (int)staleDurationMs + "ms)";
            isHolding = false;
            pendingHold = false;
        }

        // === FALLBACK MODE: Basic drive only ===
        if (fallbackMode) {
            double forward = -gamepad.left_stick_y;
            double strafe = gamepad.left_stick_x;
            double turn = gamepad.right_trigger - gamepad.left_trigger;
            forward = forward * forward * forward;
            strafe = strafe * strafe * strafe;
            turn = (turn * turn * turn) / 1.5;
            double[] powers = computeDrivePowers(forward, strafe, turn, dt);
            mecanumSubsystem.setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
            return;  // Skip all advanced logic
        }

        // === NORMAL OPERATION ===
        // Read raw velocities (assumed robot-centric X/Y in in/s, H in rad/s)

        // Low-pass filter with time constant
        double alpha = dt / (FILTER_TAU + dt);
        diagAlpha = alpha;

        filtVelX += alpha * (rawVelX - filtVelX);
        filtVelY += alpha * (rawVelY - filtVelY);
        filtVelH += alpha * (rawVelH - filtVelH);

        diagFiltVelX = filtVelX;
        diagFiltVelY = filtVelY;
        diagFiltVelH = filtVelH;

        // Read driver input
        double forward = -gamepad.left_stick_y;
        double strafe  =  gamepad.left_stick_x;
        double turn    =  gamepad.right_trigger - gamepad.left_trigger;

        // cube shaping
        forward = forward * forward * forward;
        strafe  = strafe  * strafe  * strafe;
        turn    = (turn * turn * turn) / 1.5;

        double inputMag = Math.hypot(forward, strafe) + Math.abs(turn);

        double velMag   = Math.hypot(filtVelX, filtVelY);
        double omegaMag = Math.abs(filtVelH);

        // Exit hold immediately on input (with hysteresis)
        if (isHolding && inputMag > INPUT_DEADBAND_EXIT) {
            isHolding = false;
            pendingHold = false;
            positionInDeadband = false;
            lastCorrX = lastCorrY = lastCorrH = 0;
        }

        // Hold entry gating with time delay
        if (!isHolding) {
            boolean conditionsMet =
                    inputMag < INPUT_DEADBAND_ENTER &&
                    velMag   < VELOCITY_THRESHOLD &&
                    omegaMag < OMEGA_THRESHOLD;

            if (conditionsMet) {
                if (!pendingHold) {
                    pendingHold = true;
                    holdEntryTimerNs = nowNs;
                } else {
                    double elapsedMs = (nowNs - holdEntryTimerNs) / 1e6;
                    if (elapsedMs >= HOLD_CAPTURE_DELAY_MS) {
                        pendingHold = false;
                        isHolding = true;

                        holdX = robot.pinpointSubsystem.getXInches();
                        holdY = robot.pinpointSubsystem.getYInches();
                        holdHeadingRad = robot.pinpointSubsystem.getHeadingRadians();

                        lastCorrX = lastCorrY = lastCorrH = 0;
                        positionInDeadband = false;
                    }
                }
            } else {
                pendingHold = false;
            }
        }

        double powerFL, powerFR, powerBL, powerBR;

        if (isHolding) {
            // Current pose (field)
            double currentX = robot.pinpointSubsystem.getXInches();
            double currentY = robot.pinpointSubsystem.getYInches();
            double currentH = robot.pinpointSubsystem.getHeadingRadians();

            // FIELD errors
            double errFieldX = holdX - currentX;
            double errFieldY = holdY - currentY;
            double errH = normalizeAngle(holdHeadingRad - currentH);

            // Position deadband hysteresis on magnitude
            double posErrMag = Math.hypot(errFieldX, errFieldY);
            if (positionInDeadband && posErrMag > POSITION_DEADBAND_EXIT) {
                positionInDeadband = false;
            } else if (!positionInDeadband && posErrMag < POSITION_DEADBAND_ENTER) {
                positionInDeadband = true;
            }
            if (positionInDeadband) {
                errFieldX = 0;
                errFieldY = 0;
            }
            if (Math.abs(errH) < HEADING_DEADBAND) errH = 0;

            // Rotation angle used in your drive code: angle = -(heading - offset)
            double offset = (side != null && !side) ? Math.PI : 0.0;
            double angle = -(currentH - offset);

            // Rotate FIELD error -> ROBOT error
            double errRobotX = errFieldX * Math.cos(angle) - errFieldY * Math.sin(angle);
            double errRobotY = errFieldX * Math.sin(angle) + errFieldY * Math.cos(angle);

            // PD in ROBOT frame (velocities assumed robot-centric)
            double corrRobotX = errRobotX * HOLD_KP - filtVelX * HOLD_KD;
            double corrRobotY = errRobotY * HOLD_KP - filtVelY * HOLD_KD;
            double corrH      = errH      * HOLD_H_KP - filtVelH * HOLD_H_KD;

            // Diagnostics (errors)
            diagErrFieldX = errFieldX;
            diagErrFieldY = errFieldY;
            diagErrH = errH;
            diagErrRobotX = errRobotX;
            diagErrRobotY = errRobotY;

            // Clamp XY vector in robot frame
            diagXyClamped = false;
            double corrMag = Math.hypot(corrRobotX, corrRobotY);
            if (corrMag > MAX_HOLD_XY_POWER && corrMag > 1e-6) {
                double s = MAX_HOLD_XY_POWER / corrMag;
                corrRobotX *= s;
                corrRobotY *= s;
                diagXyClamped = true;
            }

            // Clamp heading
            diagHClamped = false;
            double corrHUnclamped = corrH;
            corrH = Math.max(-MAX_HOLD_H_POWER, Math.min(MAX_HOLD_H_POWER, corrH));
            if (Math.abs(corrH) < Math.abs(corrHUnclamped)) diagHClamped = true;

            // Slew limit XY (delta vector)
            diagSlewXyActive = false;
            double dx = corrRobotX - lastCorrX;
            double dy = corrRobotY - lastCorrY;
            double dMag = Math.hypot(dx, dy);
            double maxD = SLEW_RATE_PER_SEC * dt;
            if (dMag > maxD && dMag > 1e-6) {
                double s = maxD / dMag;
                dx *= s;
                dy *= s;
                diagSlewXyActive = true;
            }
            corrRobotX = lastCorrX + dx;
            corrRobotY = lastCorrY + dy;
            lastCorrX = corrRobotX;
            lastCorrY = corrRobotY;

            // Slew limit heading
            diagSlewHActive = false;
            double dH = corrH - lastCorrH;
            double maxDH = SLEW_RATE_H_PER_SEC * dt;
            if (Math.abs(dH) > maxDH) {
                dH = Math.signum(dH) * maxDH;
                diagSlewHActive = true;
            }
            corrH = lastCorrH + dH;
            lastCorrH = corrH;

            // Diagnostics (final corrections)
            diagCorrRobotX = corrRobotX;
            diagCorrRobotY = corrRobotY;
            diagCorrH = corrH;

            // Explicit axis mapping: robot X = forward, robot Y = left strafe
            double corrForward = corrRobotX;
            double corrStrafe  = corrRobotY;

            // Mecanum mix
            powerFL = corrForward + corrStrafe + corrH;
            powerFR = corrForward - corrStrafe - corrH;
            powerBL = corrForward - corrStrafe + corrH;
            powerBR = corrForward + corrStrafe - corrH;

            // Normalize if needed
            double maxPower = Math.max(1.0, Math.max(
                    Math.max(Math.abs(powerFL), Math.abs(powerFR)),
                    Math.max(Math.abs(powerBL), Math.abs(powerBR))
            ));
            powerFL /= maxPower;
            powerFR /= maxPower;
            powerBL /= maxPower;
            powerBR /= maxPower;

        } else {
            // Driving
            double[] p = computeDrivePowers(forward, strafe, turn, dt);
            powerFL = p[0];
            powerFR = p[1];
            powerBL = p[2];
            powerBR = p[3];

            // Clear hold diagnostics
            diagErrFieldX = diagErrFieldY = diagErrH = 0;
            diagErrRobotX = diagErrRobotY = 0;
            diagCorrRobotX = diagCorrRobotY = diagCorrH = 0;
            diagXyClamped = diagHClamped = false;
            diagSlewXyActive = diagSlewHActive = false;
        }

        mecanumSubsystem.setMotorPowers(powerFL, powerFR, powerBL, powerBR);
    }

    // === Diagnostic getters ===
    public boolean isHolding() { return isHolding; }
    public double getHoldX() { return holdX; }
    public double getHoldY() { return holdY; }
    public double getHoldHeadingRad() { return holdHeadingRad; }

    public double getFiltVelX() { return diagFiltVelX; }
    public double getFiltVelY() { return diagFiltVelY; }
    public double getFiltVelH() { return diagFiltVelH; }
    public double getErrFieldX() { return diagErrFieldX; }
    public double getErrFieldY() { return diagErrFieldY; }
    public double getErrH() { return diagErrH; }
    public double getErrRobotX() { return diagErrRobotX; }
    public double getErrRobotY() { return diagErrRobotY; }
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

    // Fallback mode getters
    public boolean isFallbackMode() { return fallbackMode; }
    public String getFallbackReason() { return fallbackReason; }
    public void setFallbackMode(boolean enabled) { 
        fallbackMode = enabled; 
        fallbackReason = enabled ? "External" : "";
        if (enabled) { isHolding = false; pendingHold = false; }
    }

    protected double normalizeAngle(double angle) {
        if (Double.isNaN(angle) || Double.isInfinite(angle)) return 0;
        return Math.IEEEremainder(angle, 2 * Math.PI);
    }

    /**
     * Compute drive motor powers when NOT holding position.
     * Override in subclasses to add anti-push velocity correction.
     */
    protected double[] computeDrivePowers(double forward, double strafe, double turn, double dt) {
        if (side != null) {
            double heading = robot.pinpointSubsystem.getHeadingRadians();
            double offset = side ? 0 : Math.PI;
            double angle = -(heading - offset);

            double oldForward = forward;
            double oldStrafe = strafe;

            strafe  = oldStrafe * Math.cos(angle) - oldForward * Math.sin(angle);
            forward = oldStrafe * Math.sin(angle) + oldForward * Math.cos(angle);
        }

        double multiplier = Common.DRIVE_DEFAULT_MULT;
        double denominator = Math.max(multiplier * (Math.abs(forward) + Math.abs(strafe) + Math.abs(turn)), 1.0);

        double powerFL = multiplier * (forward + strafe + turn) / denominator;
        double powerFR = multiplier * (forward - strafe - turn) / denominator;
        double powerBL = multiplier * (forward - strafe + turn) / denominator;
        double powerBR = multiplier * (forward + strafe - turn) / denominator;

        // reset hold slew history when driving
        lastCorrX = lastCorrY = lastCorrH = 0;

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