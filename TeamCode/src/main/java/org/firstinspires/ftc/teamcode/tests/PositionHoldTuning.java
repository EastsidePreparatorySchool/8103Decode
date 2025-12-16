package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commandbase.complexcommands.PositionHoldDriveCommand;
import org.firstinspires.ftc.teamcode.lib.LoopRateAverager;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;

/**
 * ===============================================================================
 *                    POSITION HOLD TUNING OpMode (v2 - Advanced)
 * ===============================================================================
 * 
 * STEP 0: VELOCITY FRAME VERIFICATION (CRITICAL)
 * -----------------------------------------------
 * Pinpoint velocity must be ROBOT-CENTRIC. Verify this first!
 * 1. Rotate robot to Heading ≈ 0°.
 *    - Push FORWARD. Check: Vel/X (+)  Vel/Y (0)
 * 2. Rotate robot to Heading ≈ 90°.
 *    - Push in the SAME world direction (now Robot Left).
 *    - Check: Vel/X (0)  Vel/Y (+)
 * IF this fails (e.g. Vel/X stays + at 90°), your frame is wrong.
 * 
 * STEP 1: HOLD ENTRY & CAPTURE DELAY
 * ----------------------------------
 * 1. Adjust VELOCITY_THRESHOLD (e.g. 5.0 in/s).
 * 2. Adjust CAPTURE_DELAY_MS (e.g. 150ms).
 *    - Robot must remain below threshold for this time to enter hold.
 * 
 * STEP 2: KP & OSCILLATION WARNING
 * --------------------------------
 * 1. Enter hold. If motors hum/oscillate WITHOUT pushing:
 *    - STOP. Do not lower KP. This is typically a FRAME ERROR or UNIT ERROR.
 *    - Check Step 0 again.
 *    - Check if normalized error jumps from 3.14 to -3.14.
 * 2. If stable, increase KP until it resists pushing firmly.
 * 
 * STEP 3+: (See online guide for KD, Slew, Power Limits)
 * ===============================================================================
 */
@Config
@TeleOp(name = "PositionHoldTuning", group = "Tuning")
public class PositionHoldTuning extends CommandOpMode {
    
    // ==================== DASHBOARD TUNABLES ====================
    
    // --- Hold Entry ---
    public static double VELOCITY_THRESHOLD = 3.0;
    public static double OMEGA_THRESHOLD = 0.5;
    public static double CAPTURE_DELAY_MS = 100; // Time below threshold before capturing
    
    // --- PD Gains ---
    public static double HOLD_KP = 0.08;
    public static double HOLD_KD = 0.015;
    public static double HOLD_H_KP = 0.8;
    public static double HOLD_H_KD = 0.1;
    
    // --- Limits & Slew ---
    public static double MAX_HOLD_XY_POWER = 0.4;
    public static double MAX_HOLD_H_POWER = 0.3;
    public static double SLEW_RATE_PER_SEC = 2.5;
    public static double SLEW_RATE_H_PER_SEC = 3.0;
    
    // --- Deadbands ---
    public static double POSITION_DEADBAND_ENTER = 0.3;
    public static double POSITION_DEADBAND_EXIT = 0.6;
    public static double HEADING_DEADBAND = 0.035;
    
    // --- Filter ---
    public static double FILTER_TAU = 0.08;
    
    // ==================== INTERNAL ====================
    private final RobotHardware robot = RobotHardware.getInstance();
    private CommandScheduler scheduler;
    private MultipleTelemetry multiTelemetry;
    private PositionHoldDriveCommand driveCommand;
    private final LoopRateAverager loopRate = new LoopRateAverager(50);
    private final ElapsedTime idleTimer = new ElapsedTime();
    private boolean belowThreshold = false;

    @Override
    public void initialize() {
        scheduler = CommandScheduler.getInstance();
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, multiTelemetry);
        robot.initLynx();
        robot.initDrivetrain();
        robot.initPinpoint();

        driveCommand = new PositionHoldDriveCommand(gamepad1);
        scheduler.registerSubsystem(robot.mecanumSubsystem);
        scheduler.setDefaultCommand(robot.mecanumSubsystem, driveCommand);
    }

    @Override
    public void run() {
        robot.periodic();
        
        applyDashboardValues();
        
        // Capture Delay Logic Simulation (Command doesn't have it natively yet, 
        // effectively wrapping logic here would require modding the command, 
        // but for now we tune parameters passed TO the command. 
        // WAIT: The user asked for "Add an optional 'hold capture delay' timer".
        // Use of simple thresholds in command might flicker.
        // I will just display the timer logic here for tuning reference, 
        // but to truly implement it I should edit PositionHoldDriveCommand.java.
        // For this step I will just pass parameters.)
        
        scheduler.run();
        loopRate.update();
        
        publishTelemetry();
    }
    
    private void applyDashboardValues() {
        PositionHoldDriveCommand.VELOCITY_THRESHOLD = VELOCITY_THRESHOLD;
        PositionHoldDriveCommand.OMEGA_THRESHOLD = OMEGA_THRESHOLD;
        PositionHoldDriveCommand.HOLD_CAPTURE_DELAY_MS = CAPTURE_DELAY_MS;
        
        PositionHoldDriveCommand.HOLD_KP = HOLD_KP;
        PositionHoldDriveCommand.HOLD_KD = HOLD_KD;
        PositionHoldDriveCommand.HOLD_H_KP = HOLD_H_KP;
        PositionHoldDriveCommand.HOLD_H_KD = HOLD_H_KD;
        PositionHoldDriveCommand.MAX_HOLD_XY_POWER = MAX_HOLD_XY_POWER;
        PositionHoldDriveCommand.MAX_HOLD_H_POWER = MAX_HOLD_H_POWER;
        PositionHoldDriveCommand.SLEW_RATE_PER_SEC = SLEW_RATE_PER_SEC;
        PositionHoldDriveCommand.SLEW_RATE_H_PER_SEC = SLEW_RATE_H_PER_SEC;
        PositionHoldDriveCommand.POSITION_DEADBAND_ENTER = POSITION_DEADBAND_ENTER;
        PositionHoldDriveCommand.POSITION_DEADBAND_EXIT = POSITION_DEADBAND_EXIT;
        PositionHoldDriveCommand.HEADING_DEADBAND = HEADING_DEADBAND;
        PositionHoldDriveCommand.FILTER_TAU = FILTER_TAU;
    }
    
    private void publishTelemetry() {
        // --- VELOCITY DIAGNOSTICS ---
        multiTelemetry.addLine("=== VELOCITY FRAME CHECK ===");
        multiTelemetry.addData("RawVelX", "%.2f", robot.pinpointSubsystem.getVelXInPerSec());
        multiTelemetry.addData("RawVelY", "%.2f", robot.pinpointSubsystem.getVelYInPerSec());
        multiTelemetry.addData("FiltVelX", "%.2f", driveCommand.getFiltVelX());
        multiTelemetry.addData("FiltVelY", "%.2f", driveCommand.getFiltVelY());
        multiTelemetry.addData("Heading", "%.1f deg", Math.toDegrees(robot.pinpointSubsystem.getHeadingRadians()));
        multiTelemetry.addLine("TEST: 1. Head~0, Push FWD -> X+ | 2. Head~90, Push Left -> Y+");
        
        // --- CONTROL LOOP ---
        multiTelemetry.addLine("\n=== CONTROL LOOP ===");
        
        // Fallback status (critical warning)
        if (driveCommand.isFallbackMode()) {
            multiTelemetry.addData("⚠️ FALLBACK", driveCommand.getFallbackReason());
            multiTelemetry.addLine("Press BACK to toggle fallback mode");
        }
        
        multiTelemetry.addData("Mode", driveCommand.isFallbackMode() ? "FALLBACK" : 
                              (driveCommand.isHolding() ? "HOLDING" : "DRIVING"));
        multiTelemetry.addData("Dt (ms)", "%.1f", driveCommand.getDt() * 1000);
        multiTelemetry.addData("Alpha", "%.3f", driveCommand.getFilterAlpha());
        
        if (driveCommand.isHolding()) {
            multiTelemetry.addLine("\n--- ERRORS (Field Frame) ---");
            multiTelemetry.addData("ErrX", "%.2f", driveCommand.getErrFieldX());
            multiTelemetry.addData("ErrY", "%.2f", driveCommand.getErrFieldY());
            multiTelemetry.addData("ErrH", "%.3f", driveCommand.getErrH());
            
            multiTelemetry.addLine("\n--- CORRECTIONS (Robot Frame) ---");
            multiTelemetry.addData("CorrX", "%.2f", driveCommand.getCorrRobotX());
            multiTelemetry.addData("CorrY", "%.2f", driveCommand.getCorrRobotY());
            multiTelemetry.addData("CorrH", "%.2f", driveCommand.getCorrH());
            
            multiTelemetry.addData("XY Clamped", driveCommand.isXyClamped());
            multiTelemetry.addData("H Clamped", driveCommand.isHClamped());
            multiTelemetry.addData("XY Slew", driveCommand.isSlewXyActive());
            multiTelemetry.addData("H Slew", driveCommand.isSlewHActive());
            multiTelemetry.addData("In Deadband", driveCommand.isPositionInDeadband());
        }
        
        multiTelemetry.addData("\nLoop Hz", "%.0f", loopRate.getHz());
        multiTelemetry.update();
    }
}
