package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.lib.LoopRateAverager;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;

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

    @Override
    public void initialize() {
        scheduler = CommandScheduler.getInstance();
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, multiTelemetry);
        robot.initLynx();
        robot.initShooter();

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
        applyTargets();
        updateMechanism();
        publishTelemetry();
    }

    @Override
    public void run() {
        robot.periodic();
        scheduler.run();

        // Live update of tuning values
        updateShooterCoefficients();

        applyTargets();
        updateMechanism();
        publishTelemetry();
    }

    @Override
    public void end() {
        shooterSubsystem.setShooterState(ShooterSubsystem.ShooterState.OFF);
    }

    private void applyTargets() {
        shooterSubsystem.setTargetRpm(shooterTargetRpm);
        hoodSubsystem.setHoodPosition(hoodPosition);
    }

    private void updateMechanism() {
        shooterSubsystem.updateHardware();
    }

    private void publishTelemetry() {
        loopRate.update();
        multiTelemetry.addData("shooter/target rpm", shooterTargetRpm);
        multiTelemetry.addData("shooter/current rpm", shooterSubsystem.currentRpm);
        multiTelemetry.addData("shooter/error rpm", shooterTargetRpm - shooterSubsystem.currentRpm);
        multiTelemetry.addData("shooter/power", shooterSubsystem.power);
        multiTelemetry.addData("hood/pos", hoodPosition);
        multiTelemetry.addData("hz", loopRate.getHz());
        multiTelemetry.update();
    }

    private void updateShooterCoefficients() {
        shooterSubsystem.applyVelocityCoefficients(shooterKs, shooterKv, shooterKa, shooterKp);
    }
}
