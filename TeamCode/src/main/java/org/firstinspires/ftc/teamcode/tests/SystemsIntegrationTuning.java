package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.LoopRateAverager;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Disabled
@Config
@TeleOp(name = "SystemsIntegrationTuning", group = "Tuning")
public class SystemsIntegrationTuning extends CommandOpMode {
    // Dashboard-controlled inputs
    public static boolean transferUp = false;           // false = DOWN, true = UP
    public static int spindexerSlot = 1;                // 1..6 mapping to INTAKE/OUTTAKE slots
    public static double hoodPosition = Common.HOOD_INITIAL_POS; // [0..1]
    public static double turretTargetDeg = 0.0;         // arbitrary angle (wire-wrap safe applied)
    public static double shooterTargetRpm = 0.0;        // 0 = off
    public static boolean intakeOn = false;             // forward on/off

    private final RobotHardware robot = RobotHardware.getInstance();
    private CommandScheduler scheduler;
    private MultipleTelemetry multiTelemetry;
    private final LoopRateAverager loopRate = new LoopRateAverager(50);

    // Subsystems
    private ShooterSubsystem shooterSubsystem;
    private HoodSubsystem hoodSubsystem;
    private TurretSubsystem turretSubsystem;
    private SpindexerSubsystem spindexerSubsystem;
    private TransferSubsystem transferSubsystem;
    private IntakeSubsystem intakeSubsystem;

    @Override
    public void initialize() {
        scheduler = CommandScheduler.getInstance();
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, multiTelemetry);
        robot.initLynx();

        // Initialize and register subsystems available via RobotHardware helpers
        robot.initShooter();
        robot.initHood();
        robot.initTurret();
        robot.initSpindexer();
        robot.initTransfer();
        robot.initIntake();

        shooterSubsystem = robot.shooterSubsystem;
        hoodSubsystem = robot.hoodSubsystem;
        turretSubsystem = robot.turretSubsystem;
        spindexerSubsystem = robot.spindexerSubsystem;
        transferSubsystem = robot.transferSubsystem;

        // Access intake subsystem via RobotHardware
        intakeSubsystem = robot.intakeSubsystem;

        // Enable mechanisms by default; states are driven by dashboard inputs
        shooterSubsystem.setShooterState(ShooterSubsystem.ShooterState.ON);
        turretSubsystem.setTurretState(TurretSubsystem.TurretState.RUNNING);

        // Initialize targets from hardware where sensible
        hoodPosition = Common.HOOD_INITIAL_POS;
        shooterTargetRpm = 0.0;
        turretTargetDeg = turretSubsystem.ticksToDegrees(robot.turret.getCurrentPosition());
        spindexerSlot = 1; // INTAKE_ONE
        transferUp = false;
        intakeOn = false;

        // Apply initial coeffs so Common values take effect
        updateShooterCoefficients();

        publishTelemetry();
    }

    @Override
    public void initialize_loop() {
        robot.periodic();
        scheduler.run();
        applyTargets();
        updateMechanisms();
        publishTelemetry();
    }

    @Override
    public void run() {
        robot.periodic();
        scheduler.run();

        // Live updates if Common shooter coefficients are tweaked on Dashboard
        updateShooterCoefficients();

        applyTargets();
        updateMechanisms();
        publishTelemetry();
    }

    @Override
    public void end() {
        // Safe shutdown for moving parts
        shooterSubsystem.setShooterState(ShooterSubsystem.ShooterState.OFF);
        turretSubsystem.setTurretState(TurretSubsystem.TurretState.STOPPED);
        intakeSubsystem.setIntakeState(IntakeSubsystem.IntakeState.STOPPED);
    }

    private void applyTargets() {
        // Transfer: boolean toggle
        transferSubsystem.setTransferState(transferUp ? TransferSubsystem.TransferState.UP
                                                     : TransferSubsystem.TransferState.DOWN);

        // Spindexer: 1..6 mapping to states
        SpindexerSubsystem.SpindexerState s = mapSlotToSpindexerState(spindexerSlot);
        spindexerSubsystem.setSpindexerState(s);

        // Hood: clamp to [0, 1]
        double clampedHood = clamp(hoodPosition, 0.0, 1.0);
        hoodSubsystem.setHoodPosition(clampedHood);

        // Turret: set target degrees (wire-wrap limit handled in subsystem)
        turretSubsystem.setTarget(turretTargetDeg);

        // Shooter RPM: clamp to [0, SHOOTER_MAX_RPM]; ON if > 0 else OFF
        double rpm = clamp(shooterTargetRpm, 0.0, Common.SHOOTER_MAX_RPM);
        shooterSubsystem.setTargetRpm(rpm);
        shooterSubsystem.setShooterState(rpm > 0.0 ? ShooterSubsystem.ShooterState.ON
                                                   : ShooterSubsystem.ShooterState.OFF);

        // Intake: boolean forward on/off
        intakeSubsystem.setIntakeState(intakeOn ? IntakeSubsystem.IntakeState.FORWARD
                                                : IntakeSubsystem.IntakeState.STOPPED);
    }

    private void updateMechanisms() {
        // Explicitly update mechanisms that compute control each loop
        shooterSubsystem.updateHardware();
        turretSubsystem.updateHardware();
        // Spindexer/Transfer/Intake update via their periodic() through scheduler.run()
    }

    private void publishTelemetry() {
        loopRate.update();

        // Shooter
        multiTelemetry.addData("shooter/target rpm", shooterTargetRpm);
        multiTelemetry.addData("shooter/current rpm", robot.shooterSubsystem.currentRpm);
        multiTelemetry.addData("shooter/power", robot.shooterSubsystem.power);
        multiTelemetry.addData("hood/pos", hoodPosition);

        // Turret
        double turretPosTicks = robot.turret.getCurrentPosition();
        multiTelemetry.addData("turret/target (deg)", turretTargetDeg);
        multiTelemetry.addData("turret/pos (ticks)", turretPosTicks);
        multiTelemetry.addData("turret/pos (deg)", turretSubsystem.ticksToDegrees(turretPosTicks));
        multiTelemetry.addData("turret/error (ticks)", turretSubsystem.turretPIDF.getPositionError());

        // Spindexer/Transfer/Intake
        multiTelemetry.addData("spindexer/slot", spindexerSlot);
        multiTelemetry.addData("spindexer/state", spindexerSubsystem.state);
        multiTelemetry.addData("spindexer/servo pos", spindexerSubsystem.targetPosition);
        multiTelemetry.addData("transfer/up", transferUp);
        multiTelemetry.addData("intake/on", intakeOn);

        // Misc
        multiTelemetry.addData("hz", loopRate.getHz());
        multiTelemetry.update();
    }

    private void updateShooterCoefficients() {
        shooterSubsystem.applyVelocityCoefficients(
                Common.SHOOTER_KS,
                Common.SHOOTER_KV,
                Common.SHOOTER_KA,
                Common.SHOOTER_KP
        );
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
