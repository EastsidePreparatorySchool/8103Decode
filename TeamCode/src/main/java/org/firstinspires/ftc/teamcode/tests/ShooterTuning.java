package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.LoopRateLimiter;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.lib.LoopRateAverager;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.HoodSetPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.ShooterSetTargetRPMCommand;

@Config
@TeleOp(name = "ShooterTuning", group = "Tuning")
public class ShooterTuning extends CommandOpMode {
    // Targets / tunables (visible on dashboard)
    public static double shooterTargetRpm = 0.0;
    public static double hoodPosition = Common.HOOD_INITIAL_POS;

    public static double shooterKs = Common.SHOOTER_KS;
    public static double shooterKv = Common.SHOOTER_KV;
    public static double shooterKa = Common.SHOOTER_KA;
    public static double shooterKp = Common.SHOOTER_KP;

    private final RobotHardware robot = RobotHardware.getInstance();
    private CommandScheduler scheduler;
    private MultipleTelemetry multiTelemetry;
    private ShooterSubsystem shooterSubsystem;
    private HoodSubsystem hoodSubsystem;
    private final LoopRateAverager loopRate = new LoopRateAverager(50);

    // Track previous dashboard values to avoid rescheduling every loop
    private double prevShooterTargetRpm = Double.NaN;
    private double prevHoodPos = Double.NaN;
    private final LoopRateLimiter loopRateLimiter = new LoopRateLimiter(50);

    @Override
    public void initialize() {
        scheduler = CommandScheduler.getInstance();
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, multiTelemetry);
        robot.initLynx();
        robot.initShooter();
        robot.initHood();

        shooterSubsystem = robot.shooterSubsystem;
        hoodSubsystem = robot.hoodSubsystem;
        shooterSubsystem.setShooterState(ShooterSubsystem.ShooterState.ON);

        // Initialize targets from hardware
        hoodPosition = Common.HOOD_INITIAL_POS;
        shooterTargetRpm = 0.0;

        // Apply initial coefficients
        updateShooterCoefficients();
    }

    @Override
    public void initialize_loop() {
        robot.periodic();
        scheduler.run();
        handleDashboardChanges();
        publishTelemetry();
        loopRateLimiter.waitForNextLoop();
    }

    @Override
    public void run() {
        robot.periodic();
        scheduler.run();

        // Live update of tuning values (does not touch hardware state)
        updateShooterCoefficients();

        handleDashboardChanges();
        publishTelemetry();
        loopRateLimiter.waitForNextLoop();
    }

    @Override
    public void end() {
        shooterSubsystem.setShooterState(ShooterSubsystem.ShooterState.OFF);
    }

    private void handleDashboardChanges() {
        // Clamp and schedule only on change
        double clampedHood = Math.max(0.0, Math.min(1.0, hoodPosition));
        if (clampedHood != prevHoodPos) {
            prevHoodPos = clampedHood;
            schedule(new HoodSetPositionCommand(clampedHood));
        }

        double clampedRpm = Math.max(0.0, Math.min(Common.SHOOTER_MAX_RPM, shooterTargetRpm));
        if (clampedRpm != prevShooterTargetRpm) {
            prevShooterTargetRpm = clampedRpm;
            schedule(new ShooterSetTargetRPMCommand(clampedRpm));
        }
    }

    private void publishTelemetry() {
        loopRate.update();
        multiTelemetry.addData("shooter/target rpm", shooterTargetRpm);
        multiTelemetry.addData("shooter/current rpm", shooterSubsystem.currentRpm);
        multiTelemetry.addData("shooter/error rpm", shooterTargetRpm - shooterSubsystem.currentRpm);
        multiTelemetry.addData("shooter/power", shooterSubsystem.power);
        multiTelemetry.addData("shooter/within tolerance", shooterSubsystem.withinTolerance());
        multiTelemetry.addData("hood/pos", hoodPosition);
        multiTelemetry.addData("hz", loopRate.getHz());
        multiTelemetry.update();
    }

    private void updateShooterCoefficients() {
        shooterSubsystem.applyVelocityCoefficients(shooterKs, shooterKv, shooterKa, shooterKp);
    }
}
