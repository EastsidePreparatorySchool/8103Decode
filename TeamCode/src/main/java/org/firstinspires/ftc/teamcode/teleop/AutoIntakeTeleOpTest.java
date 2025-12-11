package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.commandbase.safecommands.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.SpindexerSetPositionCommand;
import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

import java.util.HashMap;
import java.util.Map;

/**
 * Test TeleOp that extends AutoShooterTeleOpTest with automatic spindexer slot
 * switching when a ball is detected by the color/distance sensor.
 */
public class AutoIntakeTeleOpTest extends AutoShooterTeleOpTest {
    
    // Auto-intake state machine
    private enum AutoIntakeState {
        IDLE,           // Intake not running or at outtake slot
        WAITING_FOR_BALL, // Intake running, at intake slot, waiting for ball
        BALL_DETECTED,  // Ball detected, reading color and switching
        SWITCHING_SLOT, // Spindexer transitioning
        SENSOR_COOLDOWN // Waiting for sensor to clear before detecting next ball
    }
    
    private AutoIntakeState autoIntakeState = AutoIntakeState.IDLE;
    
    // Ball tracking map: slot index (0-2) -> HSV values (null = empty)
    private final Map<Integer, float[]> ballMap = new HashMap<>();
    
    // Track if a spindexer command is currently running
    private boolean spindexerCommandRunning = false;
    private long spindexerCommandStartTime = 0;
    private long spindexerCommandDuration = 0;
    
    // Track spindexer state to detect outtake transitions
    private SpindexerSubsystem.SpindexerState prevSpindexerState = null;
    
    @Override
    public void initialize() {
        super.initialize();
        
        // Initialize color sensor
        robot.initColorSensor();
        
        // Initialize ball map - all slots empty
        for (int i = 0; i < 3; i++) {
            ballMap.put(i, null);
        }
        
        // Start with color sensor reading distance only (cheaper than reading both)
        robot.colorSensorSubsystem.setState(ColorSensorSubsystem.ColorSensorState.READING_DISTANCE);
    }
    
    @Override
    public void run() {
        super.run();
        
        // Detect spindexer state changes and clear ball map on outtake
        detectOuttakeAndClearBallMap();
        
        // Update auto-intake logic
        updateAutoIntake();
        
        // Display telemetry
        displayAutoIntakeTelemetry();
    }
    
    /**
     * Detect when spindexer moves to an outtake position and clear the corresponding slot.
     */
    private void detectOuttakeAndClearBallMap() {
        SpindexerSubsystem.SpindexerState currentState = robot.spindexerSubsystem.state;
        
        if (prevSpindexerState != currentState) {
            // State changed - check if we moved to an outtake slot
            switch (currentState) {
                case OUTTAKE_ONE:
                    clearSlot(0);
                    break;
                case OUTTAKE_TWO:
                    clearSlot(1);
                    break;
                case OUTTAKE_THREE:
                    clearSlot(2);
                    break;
                default:
                    // Not an outtake, do nothing
                    break;
            }
            prevSpindexerState = currentState;
        }
    }
    
    private void updateAutoIntake() {
        // Check if spindexer command has completed
        if (spindexerCommandRunning) {
            if (System.currentTimeMillis() - spindexerCommandStartTime >= spindexerCommandDuration) {
                spindexerCommandRunning = false;
                // Go to cooldown state to wait for sensor to clear before detecting next ball
                autoIntakeState = AutoIntakeState.SENSOR_COOLDOWN;
                // Resume distance reading
                robot.colorSensorSubsystem.setState(ColorSensorSubsystem.ColorSensorState.READING_DISTANCE);
            } else {
                // Still switching, don't do anything
                return;
            }
        }
        
        // Check if we should be monitoring for balls
        boolean intakeRunning = robot.intakeSubsystem.state == IntakeSubsystem.IntakeState.FORWARD;
        boolean atIntakeSlot = isAtIntakeSlot();
        
        if (!intakeRunning || !atIntakeSlot) {
            autoIntakeState = AutoIntakeState.IDLE;
            return;
        }
        
        // State machine for auto-intake
        switch (autoIntakeState) {
            case IDLE:
                // Transition to waiting for ball if conditions are met
                autoIntakeState = AutoIntakeState.WAITING_FOR_BALL;
                robot.colorSensorSubsystem.setState(ColorSensorSubsystem.ColorSensorState.READING_DISTANCE);
                break;
                
            case WAITING_FOR_BALL:
                // Check distance sensor for ball detection
                double distance = robot.colorSensorSubsystem.getDistance();
                if (distance < Common.BALL_DETECTION_DISTANCE_IN) {
                    autoIntakeState = AutoIntakeState.BALL_DETECTED;
                    // Switch to reading color (single I2C read for HSV)
                    robot.colorSensorSubsystem.setState(ColorSensorSubsystem.ColorSensorState.READING_BOTH);
                }
                break;
                
            case BALL_DETECTED:
                // Read color and record in ball map
                int currentSlot = getCurrentIntakeSlotIndex();
                float[] hsv = robot.colorSensorSubsystem.getHSV();
                ballMap.put(currentSlot, hsv.clone());
                
                // Find next empty slot
                int nextSlot = findNextEmptySlot();
                
                if (nextSlot == -1) {
                    // All slots full - stop intake
                    schedule(new IntakeStateCommand(IntakeSubsystem.IntakeState.STOPPED));
                    autoIntakeState = AutoIntakeState.IDLE;
                    robot.colorSensorSubsystem.setState(ColorSensorSubsystem.ColorSensorState.OFF);
                } else {
                    // Switch to next slot
                    SpindexerSubsystem.SpindexerState nextState = intakeStateForSlot(nextSlot);
                    SpindexerSetPositionCommand cmd = new SpindexerSetPositionCommand(nextState);
                    schedule(cmd);
                    
                    // Track command timing
                    spindexerCommandRunning = true;
                    spindexerCommandStartTime = System.currentTimeMillis();
                    spindexerCommandDuration = getWaitTime(getCurrentIntakeSlotIndex(), nextSlot);
                    
                    autoIntakeState = AutoIntakeState.SWITCHING_SLOT;
                    // Stop reading sensor during movement
                    robot.colorSensorSubsystem.setState(ColorSensorSubsystem.ColorSensorState.OFF);
                }
                break;
                
            case SWITCHING_SLOT:
                // Wait for spindexer command to complete (handled at top of method)
                break;
                
            case SENSOR_COOLDOWN:
                // Wait for sensor to read no ball (ball has passed) before detecting next ball
                // This prevents double-detection of the same ball
                double cooldownDistance = robot.colorSensorSubsystem.getDistance();
                if (cooldownDistance >= Common.BALL_DETECTION_DISTANCE_IN) {
                    // Sensor is clear, safe to start looking for next ball
                    autoIntakeState = AutoIntakeState.WAITING_FOR_BALL;
                }
                break;
        }
    }
    
    private boolean isAtIntakeSlot() {
        SpindexerSubsystem.SpindexerState state = robot.spindexerSubsystem.state;
        return state == SpindexerSubsystem.SpindexerState.INTAKE_ONE ||
               state == SpindexerSubsystem.SpindexerState.INTAKE_TWO ||
               state == SpindexerSubsystem.SpindexerState.INTAKE_THREE;
    }
    
    private int getCurrentIntakeSlotIndex() {
        switch (robot.spindexerSubsystem.state) {
            case INTAKE_ONE: return 0;
            case INTAKE_TWO: return 1;
            case INTAKE_THREE: return 2;
            default: return -1;
        }
    }
    
    private int findNextEmptySlot() {
        int currentSlot = getCurrentIntakeSlotIndex();
        // Check slots in order starting from next slot
        for (int i = 1; i <= 3; i++) {
            int slot = (currentSlot + i) % 3;
            if (ballMap.get(slot) == null) {
                return slot;
            }
        }
        return -1; // All slots full
    }
    
    private static SpindexerSubsystem.SpindexerState intakeStateForSlot(int slotIdx) {
        switch (slotIdx) {
            case 0: return SpindexerSubsystem.SpindexerState.INTAKE_ONE;
            case 1: return SpindexerSubsystem.SpindexerState.INTAKE_TWO;
            case 2: return SpindexerSubsystem.SpindexerState.INTAKE_THREE;
        }
        return SpindexerSubsystem.SpindexerState.INTAKE_ONE;
    }
    
    // Wait times for intake-to-intake transitions (from SpindexerSetPositionCommand)
    private long getWaitTime(int fromSlot, int toSlot) {
        // Simplified wait times for intake slots only
        long[][] waitTimes = {
            {0, 250, 550},   // From INTAKE_ONE
            {250, 0, 250},   // From INTAKE_TWO
            {550, 250, 0}    // From INTAKE_THREE
        };
        if (fromSlot >= 0 && fromSlot < 3 && toSlot >= 0 && toSlot < 3) {
            return waitTimes[fromSlot][toSlot];
        }
        return 750; // Default fallback
    }
    
    /**
     * Clear a slot from the ball map when a ball is outtaked.
     * Call this when transitioning to an outtake position.
     */
    protected void clearSlot(int slotIdx) {
        if (slotIdx >= 0 && slotIdx < 3) {
            ballMap.put(slotIdx, null);
        }
    }
    
    /**
     * Clear all slots in the ball map after a triple shot.
     */
    protected void clearAllSlots() {
        for (int i = 0; i < 3; i++) {
            ballMap.put(i, null);
        }
    }
    
    private void displayAutoIntakeTelemetry() {
        // Distance sensor reading
        double distance = robot.colorSensorSubsystem.getDistance();
        multiTelemetry.addData("Color Sensor Distance (in)", String.format("%.2f", distance));
        
        // Ball map display
        StringBuilder mapStr = new StringBuilder();
        for (int i = 0; i < 3; i++) {
            float[] hsv = ballMap.get(i);
            if (hsv == null) {
                mapStr.append("Slot ").append(i).append(": EMPTY");
            } else {
                mapStr.append("Slot ").append(i).append(": [H:")
                      .append(String.format("%.1f", hsv[0]))
                      .append(", S:").append(String.format("%.2f", hsv[1]))
                      .append(", V:").append(String.format("%.2f", hsv[2]))
                      .append("]");
            }
            if (i < 2) mapStr.append(" | ");
        }
        multiTelemetry.addData("Ball Map", mapStr.toString());
        multiTelemetry.addData("Auto-Intake State", autoIntakeState.toString());
    }
}
