package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;

import java.util.List;

/**
 * Subsystem for managing Limelight3A vision sensor.
 * Used for detecting ball patterns via AprilTag fiducials.
 * 
 * Pipeline mapping:
 * - Pipeline 2: GPP (Green Purple Purple)
 * - Pipeline 3: PGP (Purple Green Purple)
 * - Pipeline 4: PPG (Purple Purple Green)
 */
public class LimelightSubsystem extends SubsystemBase {
    private final RobotHardware robot = RobotHardware.getInstance();
    
    // Currently detected pattern (updated by detectBallPattern)
    private Common.BallPattern detectedPattern = Common.BallPattern.UNKNOWN;
    
    // Track if detection has completed
    private boolean patternDetected = false;
    
    public LimelightSubsystem() {
        // Constructor - Limelight is initialized in RobotHardware
    }
    
    @Override
    public void periodic() {
        // No continuous updates needed - detection is done on demand
    }
    
    /**
     * Switch the Limelight to a specific pipeline.
     * @param pipelineIndex The pipeline index to switch to
     */
    public void switchPipeline(int pipelineIndex) {
        if (robot.limelight != null) {
            robot.limelight.pipelineSwitch(pipelineIndex);
        }
    }
    
    /**
     * Check if an AprilTag fiducial is currently detected on the active pipeline.
     * @return true if at least one fiducial is detected
     */
    public boolean isFiducialDetected() {
        if (robot.limelight == null) {
            return false;
        }
        
        LLResult result = robot.limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return false;
        }
        
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        return fiducials != null && !fiducials.isEmpty();
    }
    
    /**
     * Get the ID of the first detected fiducial, or -1 if none detected.
     * @return The fiducial ID or -1
     */
    public int getDetectedFiducialId() {
        if (robot.limelight == null) {
            return -1;
        }
        
        LLResult result = robot.limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return -1;
        }
        
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials != null && !fiducials.isEmpty()) {
            return fiducials.get(0).getFiducialId();
        }
        return -1;
    }
    
    /**
     * Attempt to detect a ball pattern by scanning pipelines 2, 3, and 4.
     * This is a non-blocking single-check method meant to be called repeatedly
     * during the init loop.
     * 
     * @return The detected pattern, or UNKNOWN if none found yet
     */
    public Common.BallPattern checkForPattern() {
        if (robot.limelight == null) {
            return Common.BallPattern.UNKNOWN;
        }
        
        // Check pipelines in order: 2 (GPP), 3 (PGP), 4 (PPG)
        int[] pipelines = {
            Common.LIMELIGHT_PIPELINE_GPP,
            Common.LIMELIGHT_PIPELINE_PGP,
            Common.LIMELIGHT_PIPELINE_PPG
        };
        
        Common.BallPattern[] patterns = {
            Common.BallPattern.GPP,
            Common.BallPattern.PGP,
            Common.BallPattern.PPG
        };
        
        for (int i = 0; i < pipelines.length; i++) {
            switchPipeline(pipelines[i]);
            
            // Small delay to allow pipeline switch to take effect
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            
            if (isFiducialDetected()) {
                detectedPattern = patterns[i];
                patternDetected = true;
                return detectedPattern;
            }
        }
        
        return Common.BallPattern.UNKNOWN;
    }
    
    /**
     * Detect ball pattern with timeout. Meant to be called once during init.
     * Blocks until pattern detected or timeout.
     * 
     * @param timeoutMs Maximum time to wait for detection
     * @return The detected pattern, or UNKNOWN if timeout
     */
    public Common.BallPattern detectBallPattern(long timeoutMs) {
        if (robot.limelight == null) {
            return Common.BallPattern.UNKNOWN;
        }
        
        ElapsedTime timer = new ElapsedTime();
        
        while (timer.milliseconds() < timeoutMs) {
            Common.BallPattern result = checkForPattern();
            if (result != Common.BallPattern.UNKNOWN) {
                return result;
            }
        }
        
        return Common.BallPattern.UNKNOWN;
    }
    
    /**
     * Get the last detected pattern.
     * @return The detected pattern
     */
    public Common.BallPattern getDetectedPattern() {
        return detectedPattern;
    }
    
    /**
     * Check if a pattern has been successfully detected.
     * @return true if pattern was detected
     */
    public boolean isPatternDetected() {
        return patternDetected;
    }
    
    /**
     * Reset detection state (call before starting new detection).
     */
    public void resetDetection() {
        detectedPattern = Common.BallPattern.UNKNOWN;
        patternDetected = false;
    }
    
    /**
     * Start Limelight polling.
     */
    public void start() {
        if (robot.limelight != null) {
            robot.limelight.start();
        }
    }
    
    /**
     * Stop Limelight polling.
     */
    public void stop() {
        if (robot.limelight != null) {
            robot.limelight.stop();
        }
    }
    
    // ==================== Alliance Color Pipeline Support ====================
    
    /**
     * Switch to the blue alliance pipeline (pipeline 0).
     */
    public void switchToBlue() {
        switchPipeline(Common.LIMELIGHT_PIPELINE_BLUE);
    }
    
    /**
     * Switch to the red alliance pipeline (pipeline 1).
     */
    public void switchToRed() {
        switchPipeline(Common.LIMELIGHT_PIPELINE_RED);
    }
    
    /**
     * Switch to the appropriate alliance pipeline based on left/right side.
     * Left side = Blue alliance, Right side = Red alliance.
     * @param isLeftSide true for left side (blue), false for right side (red)
     */
    public void switchToAlliancePipeline(boolean isLeftSide) {
        if (isLeftSide) {
            switchToBlue();
        } else {
            switchToRed();
        }
    }
    
    /**
     * Get the current pipeline index.
     * @return The current pipeline index, or -1 if Limelight is null
     */
    public int getCurrentPipeline() {
        if (robot.limelight == null) {
            return -1;
        }
        return robot.limelight.getStatus().getPipelineIndex();
    }
    
    /**
     * Check if any target is detected on the current pipeline.
     * This checks for any valid result (color, fiducial, etc).
     * @return true if a valid target is detected
     */
    public boolean isTargetDetected() {
        if (robot.limelight == null) {
            return false;
        }
        
        LLResult result = robot.limelight.getLatestResult();
        return result != null && result.isValid();
    }
    
    /**
     * Get the tx (horizontal offset) of the detected target.
     * @return tx in degrees, or 0 if no target
     */
    public double getTx() {
        if (robot.limelight == null) {
            return 0;
        }
        
        LLResult result = robot.limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTx();
        }
        return 0;
    }
    
    /**
     * Get the ty (vertical offset) of the detected target.
     * @return ty in degrees, or 0 if no target
     */
    public double getTy() {
        if (robot.limelight == null) {
            return 0;
        }
        
        LLResult result = robot.limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTy();
        }
        return 0;
    }
    
    /**
     * Get the target area as a percentage of the image.
     * @return target area (0-100), or 0 if no target
     */
    public double getTargetArea() {
        if (robot.limelight == null) {
            return 0;
        }
        
        LLResult result = robot.limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTa();
        }
        return 0;
    }
    
    // ==================== Turret Offset Calculation ====================
    
    /**
     * Calculate the turret offset angle in degrees using Limelight tx and odometry.
     * 
     * The Limelight tx gives us the angle from camera center to target (AprilTag).
     * We also apply the configurable X/Y offset from the AprilTag center to get
     * the actual target position, then calculate the turret correction needed.
     * 
     * Math:
     * 1. Calculate lateral offset from tx: lateralOffset = distance * tan(tx)
     * 2. Add the X offset (inches from AprilTag center)
     * 3. Calculate correction angle: atan2(totalLateralOffset, distance)
     * 
     * @return The turret offset in degrees (positive = turret needs to rotate CCW/left)
     */
    public double getTurretOffsetDegrees() {
        if (!isTargetDetected()) {
            return 0;
        }
        
        double tx = getTx();
        
        // Get distance from turret to goal using odometry
        double distanceToGoal = getDistanceToGoalInches();
        
        if (distanceToGoal <= 0) {
            // Fallback: just return tx as the offset
            return tx;
        }
        
        // Calculate lateral offset in inches using tx angle
        double txRadians = Math.toRadians(tx);
        double lateralOffsetInches = distanceToGoal * Math.tan(txRadians);
        
        // Add the tunable X offset (inches from AprilTag center)
        // This shifts the aim point left/right of the AprilTag
        double totalLateralOffset = lateralOffsetInches + Common.LIMELIGHT_TARGET_OFFSET_X_IN;
        
        // Calculate the actual turret correction angle
        // This accounts for the geometry of turret rotation
        double turretCorrectionRadians = Math.atan2(totalLateralOffset, distanceToGoal);
        double turretCorrectionDegrees = Math.toDegrees(turretCorrectionRadians);
        
        return turretCorrectionDegrees;
    }
    
    /**
     * Get the distance from the turret to the goal in inches using odometry.
     * Uses the turret position (robot position + turret offset) and target coordinates.
     * 
     * @return Distance in inches, or 0 if position data unavailable
     */
    public double getDistanceToGoalInches() {
        if (robot.turretSubsystem == null) {
            return 0;
        }
        
        // Turret position is calculated in TurretSubsystem.periodic()
        double turretX = robot.turretSubsystem.turretX;
        double turretY = robot.turretSubsystem.turretY;
        
        double targetX = Common.TARGET_X_IN;
        double targetY = Common.TARGET_Y_IN;
        
        double dx = targetX - turretX;
        double dy = targetY - turretY;
        
        return Math.hypot(dx, dy);
    }
    
    /**
     * Check if the turret is aligned with the target within a tolerance.
     * Uses Limelight tx to determine alignment.
     * 
     * @param toleranceDeg The tolerance in degrees
     * @return true if |tx| <= toleranceDeg
     */
    public boolean isTurretAligned(double toleranceDeg) {
        if (!isTargetDetected()) {
            return false;
        }
        return Math.abs(getTx()) <= toleranceDeg;
    }
    
    /**
     * Check if turret is aligned using default tolerance from Common.TURRET_TOLERANCE_DEG.
     * @return true if aligned within tolerance
     */
    public boolean isTurretAligned() {
        return isTurretAligned(Common.TURRET_TOLERANCE_DEG);
    }
}

