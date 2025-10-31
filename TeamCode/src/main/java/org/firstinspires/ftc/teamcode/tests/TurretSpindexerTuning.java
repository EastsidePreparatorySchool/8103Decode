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
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Config
@TeleOp(name = "TurretSpindexerTuning", group = "Tuning")
public class TurretSpindexerTuning extends CommandOpMode {
    public static double turretTargetDeg = 0.0;
    public static double turretKp = Common.TURRET_KP;
    public static double turretKi = Common.TURRET_KI;
    public static double turretKd = Common.TURRET_KD;
    public static double turretKf = Common.TURRET_KF;

    public static double spindexerTargetDeg = 0.0;
    public static double spindexerKp = Common.SPINDEXER_KP;
    public static double spindexerKi = Common.SPINDEXER_KI;
    public static double spindexerKd = Common.SPINDEXER_KD;
    public static double spindexerKf = Common.SPINDEXER_KF;

    private final RobotHardware robot = RobotHardware.getInstance();
    private CommandScheduler scheduler;
    private MultipleTelemetry multiTelemetry;
    private TurretSubsystem turretSubsystem;
    private SpindexerSubsystem spindexerSubsystem;
    private final LoopRateAverager loopRate = new LoopRateAverager(50);

    @Override
    public void initialize() {
        scheduler = CommandScheduler.getInstance();
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, multiTelemetry);
        robot.initLynx();
        robot.initTurret();
        robot.initSpindexer();

        turretSubsystem = robot.turretSubsystem;
        spindexerSubsystem = robot.spindexerSubsystem;

        turretSubsystem.setTurretState(TurretSubsystem.TurretState.RUNNING);
        spindexerSubsystem.setSpindexerState(SpindexerSubsystem.SpindexerState.INTAKE_ONE);

        initializeTargetsFromHardware();
        updateTurretCoefficients();
        updateSpindexerCoefficients();
    }

    @Override
    public void initialize_loop() {
        robot.periodic();
        scheduler.run();
        initializeTargetsFromHardware();
        applyTargets();
        updateMechanisms();
        publishPreStartTelemetry();
    }

    @Override
    public void run() {
        robot.periodic();
        scheduler.run();

        updateTurretCoefficients();
        updateSpindexerCoefficients();

        applyTargets();
        updateMechanisms();
        publishActiveTelemetry();
    }

    @Override
    public void end() {
        turretSubsystem.setTurretState(TurretSubsystem.TurretState.STOPPED);
        robot.spindexer.setPower(0.0);
    }

    private void initializeTargetsFromHardware() {
        double currentTurretTicks = robot.turret.getCurrentPosition();
        turretTargetDeg = turretSubsystem.ticksToDegrees(currentTurretTicks);

        double spindexerVoltage = robot.spindexerAnalog.getVoltage();
        spindexerTargetDeg = spindexerSubsystem.voltageToDegrees(spindexerVoltage);
    }

    private void applyTargets() {
        turretSubsystem.setTarget(turretTargetDeg);
        spindexerSubsystem.setTargetDegrees(spindexerTargetDeg);
    }

    private void updateMechanisms() {
        turretSubsystem.updateHardware();
        spindexerSubsystem.updateHardware();
    }

    private void publishPreStartTelemetry() {
        loopRate.update();
        double turretStartTicks = robot.turret.getCurrentPosition();
        multiTelemetry.addData("turret/start pos (ticks)", turretStartTicks);
        multiTelemetry.addData("turret/start pos (deg)", turretSubsystem.ticksToDegrees(turretStartTicks));
        multiTelemetry.addData("spindexer/start deg", spindexerTargetDeg);
        multiTelemetry.addData("spindexer/current deg", spindexerSubsystem.posDegrees);
        multiTelemetry.addData("hz", loopRate.getHz());
        multiTelemetry.update();
    }

    private void publishActiveTelemetry() {
        loopRate.update();
        double turretError = turretSubsystem.turretPIDF.getPositionError();
        double spindexerError = spindexerSubsystem.spindexerPIDF.getPositionError();

        multiTelemetry.addData("turret/pos (ticks)", turretSubsystem.pos);
        multiTelemetry.addData("turret/pos (deg)", turretSubsystem.ticksToDegrees(turretSubsystem.pos));
        multiTelemetry.addData("turret/target (ticks)", turretSubsystem.target);
        multiTelemetry.addData("turret/target (deg)", turretTargetDeg);
        multiTelemetry.addData("turret/error (ticks)", turretError);
        multiTelemetry.addData("spindexer/pos (deg)", spindexerSubsystem.posDegrees);
        multiTelemetry.addData("spindexer/pos (volts)", spindexerSubsystem.posVoltage);
        multiTelemetry.addData("spindexer/target (deg)", spindexerTargetDeg);
        multiTelemetry.addData("spindexer/error (deg)", spindexerError);
        multiTelemetry.addData("hz", loopRate.getHz());
        multiTelemetry.update();
    }

    

    private void updateTurretCoefficients() {
        if (turretSubsystem.turretPIDF.getP() != turretKp
                || turretSubsystem.turretPIDF.getI() != turretKi
                || turretSubsystem.turretPIDF.getD() != turretKd
                || turretSubsystem.turretPIDF.getF() != turretKf) {
            turretSubsystem.turretPIDF.setPIDF(turretKp, turretKi, turretKd, turretKf);
        }
    }

    private void updateSpindexerCoefficients() {
        if (spindexerSubsystem.spindexerPIDF.getP() != spindexerKp
                || spindexerSubsystem.spindexerPIDF.getI() != spindexerKi
                || spindexerSubsystem.spindexerPIDF.getD() != spindexerKd
                || spindexerSubsystem.spindexerPIDF.getF() != spindexerKf) {
            spindexerSubsystem.spindexerPIDF.setPIDF(spindexerKp, spindexerKi, spindexerKd, spindexerKf);
        }
    }
}
