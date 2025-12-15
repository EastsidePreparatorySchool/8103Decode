package org.firstinspires.ftc.teamcode.commandbase.complexcommands;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Full anti-push drive command with feedforward + feedback velocity control.
 * Extends PositionHoldDriveCommand - inherits position hold, overrides driving.
 * 
 * Features:
 * - kS + kV feedforward model
 * - Velocity error correction (feedback)
 * - Base + correction split (correction clamped, not total)
 * - Inherits position hold from parent
 */
@Config
public class AntiPushDriveCommand extends PositionHoldDriveCommand {

    // === FEEDFORWARD (kS + kV model) ===
    public static double KS_X = 0.04;    // static friction compensation
    public static double KV_X = 0.0115;  // power per in/s (≈ 1/maxVel)
    public static double KS_Y = 0.05;    // strafe has more friction
    public static double KV_Y = 0.0145;
    public static double KS_H = 0.025;
    public static double KV_H = 0.22;    // power per rad/s

    // === FEEDBACK (small corrections on top of FF) ===
    public static double KP_VEL = 0.0008;   // power per in/s error
    public static double KP_OMEGA = 0.015;  // power per rad/s error

    // === LIMITS ===
    public static double MAX_CORR_XY = 0.25;  // max XY correction power
    public static double MAX_CORR_H = 0.15;   // max heading correction power

    // === MAX VELOCITIES (from Pedro Pathing Constants) ===
    // Using established robot constants for consistency
    public static double MAX_VEL_X = Constants.driveConstants.getXVelocity();   // 81.14 in/s
    public static double MAX_VEL_Y = Constants.driveConstants.getYVelocity();   // 64.14 in/s
    public static double MAX_OMEGA = 4.36;   // rad/s (~250 deg/s)

    public AntiPushDriveCommand(Gamepad gamepad, Boolean side) {
        super(gamepad, side);
    }

    public AntiPushDriveCommand(Gamepad gamepad) {
        super(gamepad, null);
    }

    /**
     * Override driving to add feedforward + velocity feedback.
     * Position hold is inherited from parent class.
     * 
     * NOTE: Pinpoint velocity (filtVelX/Y) is ROBOT-centric, so no rotation needed.
     */
    @Override
    protected double[] computeDrivePowers(double forward, double strafe, double turn, double dt) {
        double heading = robot.pinpointSubsystem.getHeadingRadians();

        // Rotate field-centric inputs to robot frame BEFORE computing expectations
        // This ensures desVel and actVel are both in Robot Frame
        if (side != null) {
            double offset = side ? 0 : Math.PI;
            double angle = -(heading - offset);
            double oldForward = forward;
            double oldStrafe = strafe;
            strafe = oldStrafe * Math.cos(angle) - oldForward * Math.sin(angle);
            forward = oldStrafe * Math.sin(angle) + oldForward * Math.cos(angle);
        }

        // Desired velocity (robot frame)
        double desVelX = forward * MAX_VEL_X;
        double desVelY = strafe * MAX_VEL_Y;
        double desOmega = turn * MAX_OMEGA;

        // Actual velocity from Pinpoint (already robot-centric, no rotation needed)
        double actVelRobotX = filtVelX;
        double actVelRobotY = filtVelY;
        double actOmega = filtVelH;

        // Velocity error (robot frame)
        double errVelX = desVelX - actVelRobotX;
        double errVelY = desVelY - actVelRobotY;
        double errOmega = desOmega - actOmega;

        // Feedforward: kS*sign(v) + kV*v
        double baseX = (Math.abs(desVelX) > 0.01) ? KS_X * Math.signum(desVelX) + KV_X * desVelX : 0;
        double baseY = (Math.abs(desVelY) > 0.01) ? KS_Y * Math.signum(desVelY) + KV_Y * desVelY : 0;
        double baseH = (Math.abs(desOmega) > 0.01) ? KS_H * Math.signum(desOmega) + KV_H * desOmega : 0;

        // Feedback correction
        double corrX = KP_VEL * errVelX;
        double corrY = KP_VEL * errVelY;
        double corrH = KP_OMEGA * errOmega;

        // Vector clamp on correction only
        double corrMag = Math.hypot(corrX, corrY);
        if (corrMag > MAX_CORR_XY) {
            double scale = MAX_CORR_XY / corrMag;
            corrX *= scale;
            corrY *= scale;
        }
        corrH = Math.max(-MAX_CORR_H, Math.min(MAX_CORR_H, corrH));

        // Slew limit on correction delta (XY)
        double deltaX = corrX - lastCorrX;
        double deltaY = corrY - lastCorrY;
        double deltaMag = Math.hypot(deltaX, deltaY);
        double maxDelta = SLEW_RATE_PER_SEC * dt;
        if (deltaMag > maxDelta && deltaMag > 0.001) {
            double scale = maxDelta / deltaMag;
            deltaX *= scale;
            deltaY *= scale;
        }
        corrX = lastCorrX + deltaX;
        corrY = lastCorrY + deltaY;
        lastCorrX = corrX;
        lastCorrY = corrY;
        
        // Slew limit on Heading (Fixed)
        double deltaH = corrH - lastCorrH;
        double maxDeltaH = SLEW_RATE_H_PER_SEC * dt;
        if (Math.abs(deltaH) > maxDeltaH) {
            deltaH = Math.signum(deltaH) * maxDeltaH;
        }
        corrH = lastCorrH + deltaH;
        lastCorrH = corrH;

        // Combine base + correction
        double finalX = baseX + corrX;
        double finalY = baseY + corrY;
        double finalH = baseH + corrH;

        // No rotation here (already handled at input stage)

        // Explicit Axis Mapping
        double driveForward = finalX;
        double driveStrafe = finalY;

        // Mecanum kinematics
        double powerFL = driveForward + driveStrafe + finalH;
        double powerFR = driveForward - driveStrafe - finalH;
        double powerBL = driveForward - driveStrafe + finalH;
        double powerBR = driveForward + driveStrafe - finalH;

        // Normalize if saturated
        double maxPower = Math.max(Math.abs(powerFL), Math.max(Math.abs(powerFR),
                Math.max(Math.abs(powerBL), Math.abs(powerBR))));
        if (maxPower > 1.0) {
            powerFL /= maxPower;
            powerFR /= maxPower;
            powerBL /= maxPower;
            powerBR /= maxPower;
        }

        return new double[]{powerFL, powerFR, powerBL, powerBR};
    }
}
