package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.PerpetualCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.AimTurretAtPointCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.AutoHoodPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.AutoShooterRPMCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.TripleShotCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.ShooterStateCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.SpindexerSetPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.TurretStateCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.DistanceSensorStateCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.ColorSensorStateCommand;
import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.LoopRateAverager;
import org.firstinspires.ftc.teamcode.lib.PersistentState;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/**
 * TeleOp with automatic spindexer slot switching based on distance sensor ball detection.
 * Tracks ball occupancy and HSV colors, disables intake when full, clears on tripleshot.
 */
public class AutoIntakeTeleOp extends CommandOpMode {
    public final RobotHardware robot = RobotHardware.getInstance();
    public CommandScheduler scheduler;
    public MultipleTelemetry multiTelemetry;

    // Default commands
    public AimTurretAtPointCommand aimCommand;
    protected DriveWithGamepadCommand driveCommand;
    public AutoShooterRPMCommand shooterRPMCommand;
    public AutoHoodPositionCommand hoodPositionCommand;

    // Turret offset
    public double turretAngleOffsetDeg = 0.0;
    public double hoodOffset = 0.0;

    // Edge detection for gamepad
    public boolean prevA, prevY, prevLB, prevRB;
    public boolean prevDpadLeft, prevDpadRight, prevDpadUp, prevDpadDown;

    // Loop rate tracking
    private final LoopRateAverager loopRateAverager = new LoopRateAverager();

    // Ball tracking: HSV values for each slot (null if empty)
    // slotHSV[0] = slot 1, slotHSV[1] = slot 2, slotHSV[2] = slot 3
    private final float[][] slotHSV = {null, null, null};

    // Detection state machine
    private enum DetectionState {
        IDLE,             // Waiting for ball detection
        SPINDEXER_MOVING, // Spindexer moving, ignore sensor
        COOLDOWN          // After spindexer settles, wait 100ms before reading
    }
    private DetectionState detectionState = DetectionState.IDLE;
    private ElapsedTime cooldownTimer = new ElapsedTime();
    private static final long COOLDOWN_MS = 100; // Extra safety margin after spindexer wait

    // Track previous distance sensor state for edge detection
    private boolean prevDistanceWithin50MM = false;

    // Track if we're currently switching slots
    private boolean slotSwitchInProgress = false;
    private long slotSwitchStartTime = 0;
    private long slotSwitchWaitTime = 0;

    // Track if tripleshot is in progress (need to wait for return to INTAKE_ONE)
    private boolean tripleShotInProgress = false;
    private SpindexerSubsystem.SpindexerState lastKnownSpindexerState = null;

    // Wait times matrix from SpindexerSetPositionCommand (copied for reference)
    private static final long[][] WAIT_TIMES = {
            {0, 250, 550, 500, 700, 125},   // From INTAKE_ONE
            {250, 0, 250, 125, 500, 125},   // From INTAKE_TWO
            {550, 250, 0, 125, 125, 500},   // From INTAKE_THREE
            {500, 125, 125, 0, 250, 250},   // From OUTTAKE_ONE
            {700, 500, 125, 250, 0, 550},   // From OUTTAKE_TWO
            {125, 125, 500, 250, 550, 0}    // From OUTTAKE_THREE
    };

    @Override
    public void initialize() {
        scheduler = CommandScheduler.getInstance();
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, multiTelemetry);
        robot.initLynx();
        robot.initDrivetrain();
        robot.initTurret();
        robot.initShooter();
        robot.initHood();
        robot.initIntake();
        robot.initTransfer();
        robot.initSpindexer();
        robot.initDistanceSensor();
        robot.initColorSensor();

        // Pinpoint IMU handling
        Common.PINPOINT_RESET_IMU_ON_INIT = true;
        if (PersistentState.hasSavedPose) {
            Common.PINPOINT_RESET_IMU_ON_INIT = false;
        }
        robot.initPinpoint();

        initDriveCommand();
        aimCommand = new AimTurretAtPointCommand(Common.TARGET_X_IN, Common.TARGET_Y_IN);
        shooterRPMCommand = new AutoShooterRPMCommand(robot.shooterSubsystem);
        hoodPositionCommand = new AutoHoodPositionCommand(robot.hoodSubsystem);

        scheduler.setDefaultCommand(robot.mecanumSubsystem, driveCommand);
        scheduler.setDefaultCommand(robot.turretSubsystem, aimCommand);
        schedule(new PerpetualCommand(shooterRPMCommand));
        schedule(new PerpetualCommand(hoodPositionCommand));

        schedule(new TurretStateCommand(TurretSubsystem.TurretState.RUNNING));
        schedule(new ShooterStateCommand(ShooterSubsystem.ShooterState.OFF));

        // Enable distance sensor for ball detection
        schedule(new DistanceSensorStateCommand(DistanceSensorSubsystem.DistanceSensorState.ON));

        if (PersistentState.hasSavedPose) {
            Common.START_X_IN = PersistentState.savedXInches;
            Common.START_Y_IN = PersistentState.savedYInches;
            Common.START_HEADING_DEG = PersistentState.savedHeadingDeg;
        }

        if (PersistentState.hasSavedTurret) {
            robot.turretSubsystem.setTurretAngle(PersistentState.savedTurretDegrees);
        }

        // Initialize detection state
        detectionState = DetectionState.IDLE;
        clearAllSlots();
    }

    @Override
    public void initialize_loop() {
        robot.periodic();
        robot.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, Common.START_X_IN, Common.START_Y_IN, AngleUnit.DEGREES, Common.START_HEADING_DEG));
        aimCommand.setTargetPoint(Common.TARGET_X_IN, Common.TARGET_Y_IN);
        aimCommand.setAngleOffsetDegrees(turretAngleOffsetDeg);
        multiTelemetry.addData("aim target x (in)", Common.TARGET_X_IN);
        multiTelemetry.addData("aim target y (in)", Common.TARGET_Y_IN);
        multiTelemetry.update();
    }

    @Override
    public void run() {
        robot.periodic();
        scheduler.run();
        loopRateAverager.update();

        boolean shooterWithinTolerance = robot.shooterSubsystem.withinTolerance();

        // Keep aim target and offset updated each loop
        aimCommand.setTargetPoint(Common.TARGET_X_IN, Common.TARGET_Y_IN);
        aimCommand.setAngleOffsetDegrees(turretAngleOffsetDeg);

        // Handle spindexer movement state tracking
        updateDetectionState();

        // Process ball detection (only in IDLE state)
        if (detectionState == DetectionState.IDLE) {
            processBallDetection();
        }

        // Gamepad1 A: toggle intake on/off (manual override - always allowed)
        boolean a = gamepad1.a;
        if (a && !prevA) {
            IntakeSubsystem.IntakeState next = (robot.intakeSubsystem.state == IntakeSubsystem.IntakeState.FORWARD)
                    ? IntakeSubsystem.IntakeState.STOPPED
                    : IntakeSubsystem.IntakeState.FORWARD;
            schedule(new IntakeStateCommand(next));
        }
        prevA = a;

        // Gamepad1 Y: cycle through intake positions 1 -> 2 -> 3 -> 1 (manual override)
        // Only allow if not currently switching or in tripleshot
        boolean y = gamepad1.y;
        if (y && !prevY && !slotSwitchInProgress && !tripleShotInProgress) {
            int currIdx = intakeIndexFromState(robot.spindexerSubsystem.state);
            int nextIdx = (currIdx == -1) ? 0 : (currIdx + 1) % 3;
            startSlotSwitch(intakeStateForSlot(nextIdx));
        }
        prevY = y;

        // Gamepad2 left bumper: triple shot sequence
        boolean lb = gamepad2.left_bumper;
        if (lb && !prevLB) {
            if (shooterWithinTolerance && robot.shooterSubsystem.state == ShooterSubsystem.ShooterState.ON) {
                // Clear all slots before tripleshot
                clearAllSlots();
                // Mark tripleshot in progress - will track return to INTAKE_ONE
                tripleShotInProgress = true;
                detectionState = DetectionState.SPINDEXER_MOVING;
                schedule(new TripleShotCommand());
            }
        }
        prevLB = lb;

        // Gamepad2 right bumper: toggle shooter on/off
        boolean rb = gamepad2.right_bumper;
        if (rb && !prevRB) {
            ShooterSubsystem.ShooterState nextState = (robot.shooterSubsystem.state == ShooterSubsystem.ShooterState.ON)
                    ? ShooterSubsystem.ShooterState.OFF
                    : ShooterSubsystem.ShooterState.ON;
            schedule(new ShooterStateCommand(nextState));
        }
        prevRB = rb;

        // Gamepad2 Dpad left/right: adjust turret aim offset
        boolean dLeft = gamepad2.dpad_left;
        boolean dRight = gamepad2.dpad_right;
        if (dLeft && !prevDpadLeft) {
            turretAngleOffsetDeg += 2.0;
        }
        if (dRight && !prevDpadRight) {
            turretAngleOffsetDeg -= 2.0;
        }
        prevDpadLeft = dLeft;
        prevDpadRight = dRight;

        boolean dUp = gamepad2.dpad_up;
        boolean dDn = gamepad2.dpad_down;
        if (dUp && !prevDpadUp) {
            Common.HOOD_INITIAL_POS += 0.005;
            hoodOffset += 0.005;
        }
        if (dDn && !prevDpadDown) {
            Common.HOOD_INITIAL_POS -= 0.005;
            hoodOffset -= 0.005;
        }
        prevDpadUp = dUp;
        prevDpadDown = dDn;

        // Auto-disable intake when all slots are full
        if (isAllSlotsFull() && robot.intakeSubsystem.state == IntakeSubsystem.IntakeState.FORWARD) {
            schedule(new IntakeStateCommand(IntakeSubsystem.IntakeState.STOPPED));
        }

        // Telemetry
        multiTelemetry.addData("Loop Rate (Hz)", loopRateAverager.getHz());
        multiTelemetry.addData("Slot 1", formatSlotHSV(0));
        multiTelemetry.addData("Slot 2", formatSlotHSV(1));
        multiTelemetry.addData("Slot 3", formatSlotHSV(2));
        multiTelemetry.addData("All Slots Full", isAllSlotsFull());
        multiTelemetry.addData("Detection State", detectionState.toString());

        multiTelemetry.addData("pose x (in)", robot.pinpointSubsystem.getXInches());
        multiTelemetry.addData("pose y (in)", robot.pinpointSubsystem.getYInches());
        multiTelemetry.addData("heading (deg)", robot.pinpointSubsystem.getHeadingDegrees());
        multiTelemetry.addData("shooter rpm target", robot.shooterSubsystem.targetRpm);
        multiTelemetry.addData("shooter within tolerance", shooterWithinTolerance);
        multiTelemetry.addData("hood pos", robot.hoodSubsystem.hoodPos);
        multiTelemetry.addData("turret offset (deg)", turretAngleOffsetDeg);
        multiTelemetry.addData("hood offset", hoodOffset);

        double distance = Math.hypot(Common.TARGET_X_IN - robot.pinpointSubsystem.getXInches(),
                Common.TARGET_Y_IN - robot.pinpointSubsystem.getYInches());
        multiTelemetry.addData("Distance to Goal", distance);

        multiTelemetry.addData("turret target (deg)", robot.turretSubsystem.deg);
        multiTelemetry.addData("turret actual (deg)", robot.turretSubsystem.ticksToDegrees(robot.turretSubsystem.pos));
        multiTelemetry.addData("turret power calc", robot.turretSubsystem.power);
        multiTelemetry.update();
    }

    /**
     * Update the detection state machine based on spindexer movement.
     */
    private void updateDetectionState() {
        SpindexerSubsystem.SpindexerState currentSpindexerState = robot.spindexerSubsystem.state;

        // Track tripleshot completion: detect when spindexer returns to INTAKE_ONE
        if (tripleShotInProgress) {
            if (currentSpindexerState == SpindexerSubsystem.SpindexerState.INTAKE_ONE
                    && lastKnownSpindexerState != SpindexerSubsystem.SpindexerState.INTAKE_ONE) {
                // Tripleshot just returned to INTAKE_ONE, start settling wait
                tripleShotInProgress = false;
                // Use OUTTAKE_THREE -> INTAKE_ONE wait time + cooldown
                slotSwitchWaitTime = WAIT_TIMES[5][0] + COOLDOWN_MS;
                slotSwitchStartTime = System.currentTimeMillis();
                slotSwitchInProgress = true;
            }
            lastKnownSpindexerState = currentSpindexerState;
            detectionState = DetectionState.SPINDEXER_MOVING;
            return;
        }

        lastKnownSpindexerState = currentSpindexerState;

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
     * Process ball detection using distance sensor edge detection.
     * Only called when in IDLE state.
     */
    private void processBallDetection() {
        // Only process when in an intake state
        int currentSlot = getCurrentIntakeSlotIndex();
        if (currentSlot == -1) {
            prevDistanceWithin50MM = robot.distanceSensorSubsystem.getWithin50MM();
            return;
        }

        boolean currentWithin50MM = robot.distanceSensorSubsystem.getWithin50MM();

        // Edge detection: false -> true means ball entered
        if (currentWithin50MM && !prevDistanceWithin50MM) {
            // Ball detected! Only mark if slot is empty
            if (slotHSV[currentSlot] == null) {
                onBallDetected(currentSlot);
            }
        }

        prevDistanceWithin50MM = currentWithin50MM;
    }

    /**
     * Called when a ball is detected. Reads color and switches to next empty slot.
     */
    private void onBallDetected(int slotIndex) {
        // Read color from color sensor (single I2C read)
        float[] hsv = readColorSensorHSV();
        slotHSV[slotIndex] = hsv;

        // Switch to next empty slot
        switchToNextEmptySlot();
    }

    /**
     * Read HSV values directly from the color sensor.
     * This is called only when a ball is detected to minimize I2C reads.
     */
    private float[] readColorSensorHSV() {
        // Read RGB directly from hardware
        int red = robot.colorSensor.red();
        int green = robot.colorSensor.green();
        int blue = robot.colorSensor.blue();

        // Convert to HSV
        float[] hsv = new float[3];
        Color.RGBToHSV(red, green, blue, hsv);
        return hsv;
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
            if (slotHSV[checkSlot] == null) {
                startSlotSwitch(intakeStateForSlot(checkSlot));
                return;
            }
        }
        // All slots full - intake will be auto-disabled in run()
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

        schedule(new SpindexerSetPositionCommand(targetState));
    }

    /**
     * Clear all slot tracking (called before tripleshot).
     */
    private void clearAllSlots() {
        slotHSV[0] = null;
        slotHSV[1] = null;
        slotHSV[2] = null;
    }

    /**
     * Check if all 3 slots are occupied.
     */
    private boolean isAllSlotsFull() {
        return slotHSV[0] != null && slotHSV[1] != null && slotHSV[2] != null;
    }

    /**
     * Get current intake slot index (0, 1, 2) or -1 if in outtake state.
     */
    private int getCurrentIntakeSlotIndex() {
        return intakeIndexFromState(robot.spindexerSubsystem.state);
    }

    /**
     * Format slot HSV for telemetry.
     */
    private String formatSlotHSV(int slotIdx) {
        if (slotHSV[slotIdx] == null) {
            return "EMPTY";
        }
        float[] hsv = slotHSV[slotIdx];
        return String.format("H:%.0f S:%.2f V:%.2f", hsv[0], hsv[1], hsv[2]);
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

    protected void initDriveCommand() {
        driveCommand = new DriveWithGamepadCommand(gamepad1);
    }
}
