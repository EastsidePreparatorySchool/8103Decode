package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commandbase.safecommands.HoodSetPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.MecanumPowerMotorsCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.PinpointSetPoseCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.ShooterSetTargetRPMCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.ShooterStateCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.SpindexerSetPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.TransferSetPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.TurretSetTargetCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.TurretStateCommand;
import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.LoopRateAverager;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Config
@TeleOp(name = "SystemsIntegrationSafeCommands", group = "Tuning")
public class SystemsIntegrationSafeCommands extends CommandOpMode {
    // Dashboard-controlled inputs
    public static boolean transferUp = false;              // false = DOWN, true = UP
    public static int spindexerSlot = 1;                   // 1..6 mapping to INTAKE/OUTTAKE slots
    public static double hoodPosition = Common.HOOD_INITIAL_POS; // [0..1]
    public static boolean turretRunning = true;            // turret RUNNING/STOPPED
    public static double turretTargetDeg = 0.0;            // arbitrary target angle (wire-wrap handled in subsystem)
    public static boolean shooterOn = false;               // shooter state
    public static double shooterTargetRpm = 0.0;           // 0 = off
    public static boolean intakeOn = false;                // forward on/off

    // Drivetrain power (raw per-wheel)
    public static double driveFL = 0.0;
    public static double driveFR = 0.0;
    public static double driveBL = 0.0;
    public static double driveBR = 0.0;

    // Pinpoint pose init
    public static double PINPOINT_X_IN = 0.0;
    public static double PINPOINT_Y_IN = 0.0;
    public static double PINPOINT_HEADING_DEG = 0.0;
    public static boolean pinpointReinit = false;          // toggle to reinit pose

    private final RobotHardware robot = RobotHardware.getInstance();
    private CommandScheduler scheduler;
    private MultipleTelemetry multiTelemetry;
    private final LoopRateAverager loopRate = new LoopRateAverager(50);

    // Previous values to avoid re-scheduling identical commands every loop
    private boolean prevTransferUp;
    private int prevSpindexerSlot;
    private double prevHoodPos;
    private boolean prevTurretRunning;
    private double prevTurretTargetDeg;
    private boolean prevShooterOn;
    private double prevShooterTargetRpm;
    private boolean prevIntakeOn;
    private double prevFL, prevFR, prevBL, prevBR;
    private boolean prevPinpointReinit;
    private double prevPinX, prevPinY, prevPinHeading;

    @Override
    public void initialize() {
        scheduler = CommandScheduler.getInstance();
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, multiTelemetry);
        robot.initLynx();
        robot.initDrivetrain();
        robot.initShooter();
        robot.initHood();
        robot.initTurret();
        robot.initSpindexer();
        robot.initTransfer();
        robot.initIntake();
        robot.initPinpoint();

        // Seed previous values with current dashboard state to prevent duplicate initial scheduling
        prevTransferUp = transferUp;
        prevSpindexerSlot = spindexerSlot;
        prevHoodPos = hoodPosition;
        prevTurretRunning = turretRunning;
        prevTurretTargetDeg = turretTargetDeg;
        prevShooterOn = shooterOn;
        prevShooterTargetRpm = shooterTargetRpm;
        prevIntakeOn = intakeOn;
        prevFL = driveFL; prevFR = driveFR; prevBL = driveBL; prevBR = driveBR;
        prevPinpointReinit = pinpointReinit;
        prevPinX = PINPOINT_X_IN; prevPinY = PINPOINT_Y_IN; prevPinHeading = PINPOINT_HEADING_DEG;

        // Apply initial states through safe commands
        scheduler.schedule(false, new TransferSetPositionCommand(transferUp ? TransferSubsystem.TransferState.UP
                                                           : TransferSubsystem.TransferState.DOWN));
        scheduler.schedule(false, new SpindexerSetPositionCommand(mapSlotToSpindexerState(spindexerSlot)));
        scheduler.schedule(false, new HoodSetPositionCommand(clamp(hoodPosition, 0.0, 1.0)));
        scheduler.schedule(false, new TurretStateCommand(turretRunning ? TurretSubsystem.TurretState.RUNNING
                                                      : TurretSubsystem.TurretState.STOPPED));
        scheduler.schedule(false, new TurretSetTargetCommand(turretTargetDeg));
        scheduler.schedule(false, new ShooterSetTargetRPMCommand(clamp(shooterTargetRpm, 0.0, Common.SHOOTER_MAX_RPM)));
        scheduler.schedule(false, new ShooterStateCommand(shooterOn ? ShooterSubsystem.ShooterState.ON
                                                   : ShooterSubsystem.ShooterState.OFF));
        scheduler.schedule(false, new IntakeStateCommand(intakeOn ? IntakeSubsystem.IntakeState.FORWARD
                                                 : IntakeSubsystem.IntakeState.STOPPED));
        scheduler.schedule(false, new MecanumPowerMotorsCommand(clamp(driveFL, -1.0, 1.0),
                                               clamp(driveFR, -1.0, 1.0),
                                               clamp(driveBL, -1.0, 1.0),
                                               clamp(driveBR, -1.0, 1.0)));

        if (pinpointReinit) {
            scheduler.schedule(false, new PinpointSetPoseCommand(PINPOINT_X_IN, PINPOINT_Y_IN, PINPOINT_HEADING_DEG));
        }

        publishTelemetry();
    }

    @Override
    public void initialize_loop() {
        robot.periodic();
        scheduler.run();
        handleDashboardChanges();
        publishTelemetry();
    }

    @Override
    public void run() {
        robot.periodic();
        scheduler.run();
        handleDashboardChanges();
        publishTelemetry();
    }

    @Override
    public void end() {
        // Safe shutdown with safe commands
        schedule(new ShooterStateCommand(ShooterSubsystem.ShooterState.OFF));
        schedule(new TurretStateCommand(TurretSubsystem.TurretState.STOPPED));
        schedule(new IntakeStateCommand(IntakeSubsystem.IntakeState.STOPPED));
        schedule(new MecanumPowerMotorsCommand(0.0, 0.0, 0.0, 0.0));
    }

    private void handleDashboardChanges() {
        // Transfer
        if (transferUp != prevTransferUp) {
            prevTransferUp = transferUp;
            scheduler.schedule(false, new TransferSetPositionCommand(transferUp ? TransferSubsystem.TransferState.UP
                                                               : TransferSubsystem.TransferState.DOWN));
        }

        // Spindexer slot mapping
        if (spindexerSlot != prevSpindexerSlot) {
            prevSpindexerSlot = spindexerSlot;
            scheduler.schedule(false, new SpindexerSetPositionCommand(mapSlotToSpindexerState(spindexerSlot)));
        }

        // Hood
        double clampedHood = clamp(hoodPosition, 0.0, 1.0);
        if (clampedHood != prevHoodPos) {
            prevHoodPos = clampedHood;
            scheduler.schedule(false, new HoodSetPositionCommand(clampedHood));
        }

        // Turret state
        if (turretRunning != prevTurretRunning) {
            prevTurretRunning = turretRunning;
            scheduler.schedule(false, new TurretStateCommand(turretRunning ? TurretSubsystem.TurretState.RUNNING
                                                          : TurretSubsystem.TurretState.STOPPED));
        }
        // Turret target
        if (turretTargetDeg != prevTurretTargetDeg) {
            prevTurretTargetDeg = turretTargetDeg;
            scheduler.schedule(false, new TurretSetTargetCommand(turretTargetDeg));
        }

        // Shooter state
        if (shooterOn != prevShooterOn) {
            prevShooterOn = shooterOn;
            scheduler.schedule(false, new ShooterStateCommand(shooterOn ? ShooterSubsystem.ShooterState.ON
                                                       : ShooterSubsystem.ShooterState.OFF));
        }
        // Shooter target RPM
        double rpm = clamp(shooterTargetRpm, 0.0, Common.SHOOTER_MAX_RPM);
        if (rpm != prevShooterTargetRpm) {
            prevShooterTargetRpm = rpm;
            scheduler.schedule(false, new ShooterSetTargetRPMCommand(rpm));
        }

        // Intake state
        if (intakeOn != prevIntakeOn) {
            prevIntakeOn = intakeOn;
            scheduler.schedule(false, new IntakeStateCommand(intakeOn ? IntakeSubsystem.IntakeState.FORWARD
                                                     : IntakeSubsystem.IntakeState.STOPPED));
        }

        // Drivetrain powers
        double fl = clamp(driveFL, -1.0, 1.0);
        double fr = clamp(driveFR, -1.0, 1.0);
        double bl = clamp(driveBL, -1.0, 1.0);
        double br = clamp(driveBR, -1.0, 1.0);
        if (fl != prevFL || fr != prevFR || bl != prevBL || br != prevBR) {
            prevFL = fl; prevFR = fr; prevBL = bl; prevBR = br;
            scheduler.schedule(false, new MecanumPowerMotorsCommand(fl, fr, bl, br));
        }

        // Pinpoint pose init trigger or pose change while trigger held
        if (pinpointReinit && (!prevPinpointReinit
                || PINPOINT_X_IN != prevPinX || PINPOINT_Y_IN != prevPinY || PINPOINT_HEADING_DEG != prevPinHeading)) {
            scheduler.schedule(false, new PinpointSetPoseCommand(PINPOINT_X_IN, PINPOINT_Y_IN, PINPOINT_HEADING_DEG));
        }
        prevPinpointReinit = pinpointReinit;
        prevPinX = PINPOINT_X_IN; prevPinY = PINPOINT_Y_IN; prevPinHeading = PINPOINT_HEADING_DEG;
    }

    private void publishTelemetry() {
        loopRate.update();

        // Shooter
        multiTelemetry.addData("shooter/target rpm", shooterTargetRpm);
        multiTelemetry.addData("shooter/current rpm", robot.shooterSubsystem.currentRpm);
        multiTelemetry.addData("shooter/power", robot.shooterSubsystem.power);
        multiTelemetry.addData("shooter/within tolerance", robot.shooterSubsystem.isAverageRpmWithinTolerance());
        multiTelemetry.addData("hood/pos", hoodPosition);

        // Turret
        double turretPosTicks = robot.turret.getCurrentPosition();
        multiTelemetry.addData("turret/running", turretRunning);
        multiTelemetry.addData("turret/target (deg)", turretTargetDeg);
        multiTelemetry.addData("turret/pos (ticks)", turretPosTicks);
        multiTelemetry.addData("turret/pos (deg)", robot.turretSubsystem.ticksToDegrees(turretPosTicks));
        multiTelemetry.addData("turret/error (ticks)", robot.turretSubsystem.turretPIDF.getPositionError());

        // Spindexer/Transfer/Intake
        multiTelemetry.addData("spindexer/slot", spindexerSlot);
        multiTelemetry.addData("spindexer/state", robot.spindexerSubsystem.state);
        multiTelemetry.addData("spindexer/servo pos", robot.spindexerSubsystem.targetPosition);
        multiTelemetry.addData("transfer/up", transferUp);
        multiTelemetry.addData("intake/on", intakeOn);

        // Drivetrain
        multiTelemetry.addData("drive/fl", driveFL);
        multiTelemetry.addData("drive/fr", driveFR);
        multiTelemetry.addData("drive/bl", driveBL);
        multiTelemetry.addData("drive/br", driveBR);

        // Pinpoint
        multiTelemetry.addData("pinpoint/x (in)", robot.pinpointSubsystem.getXInches());
        multiTelemetry.addData("pinpoint/y (in)", robot.pinpointSubsystem.getYInches());
        multiTelemetry.addData("pinpoint/heading (deg)", robot.pinpointSubsystem.getHeadingDegrees());

        // Misc
        multiTelemetry.addData("hz", loopRate.getHz());
        multiTelemetry.update();
    }

    private static SpindexerSubsystem.SpindexerState mapSlotToSpindexerState(int slot) {
        int s = Math.max(1, Math.min(6, slot));
        switch (s) {
            case 1: return SpindexerSubsystem.SpindexerState.INTAKE_ONE;
            case 2: return SpindexerSubsystem.SpindexerState.INTAKE_TWO;
            case 3: return SpindexerSubsystem.SpindexerState.INTAKE_THREE;
            case 4: return SpindexerSubsystem.SpindexerState.OUTTAKE_ONE;
            case 5: return SpindexerSubsystem.SpindexerState.OUTTAKE_TWO;
            case 6: return SpindexerSubsystem.SpindexerState.OUTTAKE_THREE;
            default: return SpindexerSubsystem.SpindexerState.INTAKE_ONE;
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
