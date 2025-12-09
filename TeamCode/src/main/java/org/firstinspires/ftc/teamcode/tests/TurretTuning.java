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
import org.firstinspires.ftc.teamcode.lib.LoopRateLimiter;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Config
@TeleOp(name = "TurretTuning", group = "Tuning")
public class TurretTuning extends CommandOpMode {
    public static double turretTargetDeg = 0.0;
    public static double turretKp = Common.TURRET_KP;
    public static double turretKi = Common.TURRET_KI;
    public static double turretKd = Common.TURRET_KD;
    public static double turretKf = Common.TURRET_KF;
    private final RobotHardware robot = RobotHardware.getInstance();
    private CommandScheduler scheduler;
    private MultipleTelemetry multiTelemetry;
    private TurretSubsystem turretSubsystem;
    private final LoopRateAverager loopRate = new LoopRateAverager(50);
    private final LoopRateLimiter loopRateLimiter = new LoopRateLimiter(70);

    @Override
    public void initialize() {
        scheduler = CommandScheduler.getInstance();
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, multiTelemetry);
        robot.initLynx();
        robot.initTurret();

        turretSubsystem = robot.turretSubsystem;

        turretSubsystem.setTurretState(TurretSubsystem.TurretState.RUNNING);
        // Only turret tuning here

        initializeTargetsFromHardware();
        updateTurretCoefficients();
    }

    @Override
    public void initialize_loop() {
        robot.periodic();
        scheduler.run();
        initializeTargetsFromHardware();
        applyTargets();
        updateMechanisms();
        publishPreStartTelemetry();
        loopRateLimiter.waitForNextLoop();
    }

    @Override
    public void run() {
        robot.periodic();
        scheduler.run();

        updateTurretCoefficients();

        applyTargets();
        updateMechanisms();
        publishActiveTelemetry();
        loopRateLimiter.waitForNextLoop();
    }

    @Override
    public void end() {
        turretSubsystem.setTurretState(TurretSubsystem.TurretState.STOPPED);
    }

    private void initializeTargetsFromHardware() {
        double currentTurretTicks = robot.turret.getCurrentPosition();
        turretTargetDeg = turretSubsystem.ticksToDegrees(currentTurretTicks);
    }

    private void applyTargets() {
        turretSubsystem.setTarget(turretTargetDeg);
    }

    private void updateMechanisms() {
        turretSubsystem.updateHardware();
    }

    private void publishPreStartTelemetry() {
        loopRate.update();
        double turretStartTicks = robot.turret.getCurrentPosition();
        multiTelemetry.addData("turret/start pos (ticks)", turretStartTicks);
        multiTelemetry.addData("turret/start pos (deg)", turretSubsystem.ticksToDegrees(turretStartTicks));
        multiTelemetry.addData("hz", loopRate.getHz());
        multiTelemetry.update();
    }

    private void publishActiveTelemetry() {
        loopRate.update();
        double turretError = turretSubsystem.turretPIDF.getPositionError();

        multiTelemetry.addData("turret/pos (deg)", turretSubsystem.ticksToDegrees(turretSubsystem.pos));
        multiTelemetry.addData("turret/target (deg)", turretTargetDeg);
        multiTelemetry.addData("turret/error (deg)", turretSubsystem.ticksToDegrees(turretError));
        multiTelemetry.addData("turret/power", turretSubsystem.power);
        multiTelemetry.addData("hz", loopRate.getHz());
        multiTelemetry.update();
    }

    

    private void updateTurretCoefficients() {
        if (turretSubsystem.turretPIDF.getP() != turretKp
                || turretSubsystem.turretPIDF.getI() != turretKi
                || turretSubsystem.turretPIDF.getD() != turretKd
                || turretSubsystem.kf != turretKf) {
            turretSubsystem.turretPIDF.setPIDF(turretKp, turretKi, turretKd, 0);
            turretSubsystem.kf = turretKf;
        }
    }

    
}
