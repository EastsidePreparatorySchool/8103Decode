package org.firstinspires.ftc.teamcode.commandbase.complexcommands;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.Common.BallColor;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

/**
 * Command that runs intake with auto spindexer slot switching.
 * Detects balls via distance sensor, reads color, and switches slots.
 * Ends when all 3 slots are full.
 * 
 * Used in ParallelRaceGroup with FollowPathCommand for autonomous intaking.
 */
public class AutoIntakeCommand extends CommandBase {
    private final RobotHardware robot = RobotHardware.getInstance();

    // Ball tracking: BallColor for each slot (NONE if empty)
    private final BallColor[] slotColors = {BallColor.NONE, BallColor.NONE, BallColor.NONE};

    // Detection state machine
    private enum DetectionState {
        IDLE,                 // Waiting for ball detection
        AWAITING_COLOR,       // Ball detected, waiting for color confirmation
        SPINDEXER_MOVING,     // Spindexer moving, ignore sensor
        COOLDOWN              // After spindexer settles, wait before reading
    }
    private DetectionState detectionState = DetectionState.IDLE;
    private ElapsedTime cooldownTimer = new ElapsedTime();
    private ElapsedTime colorConfirmationTimer = new ElapsedTime();
    private static final long COOLDOWN_MS = 20;



    // Track the slot that is awaiting color confirmation
    private int awaitingColorSlotIndex = -1;

    // Track if we're currently switching slots
    private boolean slotSwitchInProgress = false;
    private long slotSwitchStartTime = 0;
    private long slotSwitchWaitTime = 0;

    // Wait times matrix from SpindexerSetPositionCommand
    private static final long[][] WAIT_TIMES = {
            {0, 250, 550, 500, 700, 125},   // From INTAKE_ONE
            {250, 0, 250, 125, 500, 125},   // From INTAKE_TWO
            {550, 250, 0, 125, 125, 500},   // From INTAKE_THREE
            {500, 125, 125, 0, 250, 250},   // From OUTTAKE_ONE
            {700, 500, 125, 250, 0, 550},   // From OUTTAKE_TWO
            {125, 125, 500, 250, 550, 0}    // From OUTTAKE_THREE
    };

    public AutoIntakeCommand() {
        addRequirements(robot.intakeSubsystem, robot.spindexerSubsystem, robot.distanceSensorSubsystem);
    }

    @Override
    public void initialize() {
        // Clear slot tracking
        clearAllSlots();
        
        // Directly set subsystem states instead of scheduling commands
        // (scheduling would conflict since we require these subsystems)
        robot.intakeSubsystem.setIntakeState(IntakeSubsystem.IntakeState.FORWARD);
        robot.distanceSensorSubsystem.setState(DistanceSensorSubsystem.DistanceSensorState.ON);
        
        // Ensure spindexer starts at INTAKE_ONE
        robot.spindexerSubsystem.setSpindexerState(SpindexerSubsystem.SpindexerState.INTAKE_ONE);
        
        // Reset state
        detectionState = DetectionState.IDLE;
        slotSwitchInProgress = false;
        awaitingColorSlotIndex = -1;
    }

    @Override
    public void execute() {
        // Update detection state machine
        updateDetectionState();

        // Process ball detection based on current state
        switch (detectionState) {
            case IDLE:
                processBallDetection();
                break;
            case AWAITING_COLOR:
                processColorConfirmation();
                break;
            default:
                // SPINDEXER_MOVING or COOLDOWN - do nothing
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return isAllSlotsFull();
    }

    @Override
    public void end(boolean interrupted) {
        // Turn off intake - directly set state instead of scheduling to avoid
        // infinite recursion when cancelled inside a command group
        robot.intakeSubsystem.state = IntakeSubsystem.IntakeState.STOPPED;
    }

    /**
     * Update the detection state machine based on spindexer movement.
     */
    private void updateDetectionState() {
        // Don't update state machine if we're awaiting color confirmation
        if (detectionState == DetectionState.AWAITING_COLOR) {
            return;
        }

        if (slotSwitchInProgress) {
            long elapsed = System.currentTimeMillis() - slotSwitchStartTime;
            if (elapsed >= slotSwitchWaitTime) {
                // Spindexer movement complete, enter cooldown
                slotSwitchInProgress = false;
                detectionState = DetectionState.COOLDOWN;
                cooldownTimer.reset();
            } else {
                detectionState = DetectionState.SPINDEXER_MOVING;
            }
        } else if (detectionState == DetectionState.COOLDOWN) {
            if (cooldownTimer.milliseconds() >= COOLDOWN_MS) {
                detectionState = DetectionState.IDLE;
            }
        }
    }

    /**
     * Process ball detection using distance sensor.
     * Only called when in IDLE state.
     */
    private void processBallDetection() {
        // Only process when in an intake state
        int currentSlot = getCurrentIntakeSlotIndex();
        if (currentSlot == -1) {
            return;
        }

        boolean currentWithin50MM = robot.distanceSensorSubsystem.getWithin50MM();

        // Ball detected! Only start color confirmation if slot is empty
        if (currentWithin50MM && slotColors[currentSlot] == BallColor.NONE) {
            awaitingColorSlotIndex = currentSlot;
            colorConfirmationTimer.reset();
            detectionState = DetectionState.AWAITING_COLOR;
        }
    }

    /**
     * Process color confirmation - wait for color sensor to detect purple/green
     * or timeout after configured time and mark as UNKNOWN.
     */
    private void processColorConfirmation() {
        if (awaitingColorSlotIndex < 0 || awaitingColorSlotIndex > 2) {
            detectionState = DetectionState.IDLE;
            return;
        }

        // Read color from color sensor
        BallColor detectedColor = readColorSensor();

        if (detectedColor == BallColor.PURPLE || detectedColor == BallColor.GREEN) {
            // Color confirmed! Mark slot and switch
            slotColors[awaitingColorSlotIndex] = detectedColor;
            awaitingColorSlotIndex = -1;
            switchToNextEmptySlot();
        } else if (colorConfirmationTimer.milliseconds() >= Common.COLOR_CONFIRMATION_TIMEOUT_MS) {
            // Timeout - mark as UNKNOWN and switch
            slotColors[awaitingColorSlotIndex] = BallColor.UNKNOWN;
            awaitingColorSlotIndex = -1;
            switchToNextEmptySlot();
        }
        // Otherwise keep waiting for color confirmation
    }

    /**
     * Read color sensor and determine ball color.
     */
    private BallColor readColorSensor() {
        NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();

        int red = (int) (colors.red * 255);
        int green = (int) (colors.green * 255);
        int blue = (int) (colors.blue * 255);

        float[] hsv = new float[3];
        Color.RGBToHSV(red, green, blue, hsv);
        float hue = hsv[0];
        float saturation = hsv[1];

        if (saturation < Common.COLOR_MIN_SATURATION) {
            return BallColor.UNKNOWN;
        }

        if (hue >= Common.PURPLE_HUE_MIN && hue <= Common.PURPLE_HUE_MAX) {
            return BallColor.PURPLE;
        }

        if (hue >= Common.GREEN_HUE_MIN && hue <= Common.GREEN_HUE_MAX) {
            return BallColor.GREEN;
        }

        return BallColor.UNKNOWN;
    }

    /**
     * Find and switch to the next empty intake slot.
     */
    private void switchToNextEmptySlot() {
        int currentSlot = getCurrentIntakeSlotIndex();
        if (currentSlot == -1) currentSlot = 0;

        // Find next empty slot (circular search)
        for (int i = 1; i <= 3; i++) {
            int checkSlot = (currentSlot + i) % 3;
            if (slotColors[checkSlot] == BallColor.NONE) {
                startSlotSwitch(intakeStateForSlot(checkSlot));
                return;
            }
        }
        // All slots full - isFinished() will return true
        detectionState = DetectionState.IDLE;
    }

    /**
     * Start a slot switch with proper state tracking.
     */
    private void startSlotSwitch(SpindexerSubsystem.SpindexerState targetState) {
        SpindexerSubsystem.SpindexerState currentState = robot.spindexerSubsystem.state;
        int fromIdx = getStateIndex(currentState);
        int toIdx = getStateIndex(targetState);

        if (fromIdx != -1 && toIdx != -1 && fromIdx != toIdx) {
            slotSwitchWaitTime = WAIT_TIMES[fromIdx][toIdx] + COOLDOWN_MS;
            slotSwitchStartTime = System.currentTimeMillis();
            slotSwitchInProgress = true;
            detectionState = DetectionState.SPINDEXER_MOVING;
        }

        // Directly set spindexer state instead of scheduling command
        // (scheduling would conflict since we require the spindexer subsystem)
        robot.spindexerSubsystem.setSpindexerState(targetState);
    }

    private void clearAllSlots() {
        slotColors[0] = BallColor.NONE;
        slotColors[1] = BallColor.NONE;
        slotColors[2] = BallColor.NONE;
    }

    private boolean isAllSlotsFull() {
        return slotColors[0] != BallColor.NONE && slotColors[1] != BallColor.NONE && slotColors[2] != BallColor.NONE;
    }

    private int getCurrentIntakeSlotIndex() {
        return intakeIndexFromState(robot.spindexerSubsystem.state);
    }

    private static SpindexerSubsystem.SpindexerState intakeStateForSlot(int slotIdx) {
        switch (slotIdx) {
            case 0: return SpindexerSubsystem.SpindexerState.INTAKE_ONE;
            case 1: return SpindexerSubsystem.SpindexerState.INTAKE_TWO;
            case 2: return SpindexerSubsystem.SpindexerState.INTAKE_THREE;
        }
        return SpindexerSubsystem.SpindexerState.INTAKE_ONE;
    }

    private static int intakeIndexFromState(SpindexerSubsystem.SpindexerState state) {
        switch (state) {
            case INTAKE_ONE: return 0;
            case INTAKE_TWO: return 1;
            case INTAKE_THREE: return 2;
            default: return -1;
        }
    }

    private static int getStateIndex(SpindexerSubsystem.SpindexerState state) {
        switch (state) {
            case INTAKE_ONE: return 0;
            case INTAKE_TWO: return 1;
            case INTAKE_THREE: return 2;
            case OUTTAKE_ONE: return 3;
            case OUTTAKE_TWO: return 4;
            case OUTTAKE_THREE: return 5;
            default: return -1;
        }
    }

    /** Get the slot colors array for external access (e.g., telemetry) */
    public BallColor[] getSlotColors() {
        return slotColors;
    }
}
