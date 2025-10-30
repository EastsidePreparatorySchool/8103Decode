package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Config
@TeleOp(name = "TurretSpindexerTuning", group = "Tuning")
public class TurretSpindexerTuning extends LinearOpMode {
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

    private RobotHardware robot;
    private TurretSubsystem turretSubsystem;
    private SpindexerSubsystem spindexerSubsystem;

    @Override
    public void runOpMode() {
        robot = RobotHardware.getInstance();
        MultipleTelemetry multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, multiTelemetry);
        robot.initTurret();
        robot.initSpindexer();

        turretSubsystem = new TurretSubsystem();
        spindexerSubsystem = new SpindexerSubsystem();
        robot.turretSubsystem = turretSubsystem;
        robot.spindexerSubsystem = spindexerSubsystem;

        turretSubsystem.setTurretState(TurretSubsystem.TurretState.RUNNING);
        spindexerSubsystem.setSpindexerState(SpindexerSubsystem.SpindexerState.INTAKE_ONE);

        while (!isStarted() && !isStopRequested()) {
            spindexerTargetDeg = spindexerSubsystem.voltageToDegrees(robot.spindexerAnalog.getVoltage());
            spindexerSubsystem.setTargetDegrees(spindexerTargetDeg);
            spindexerSubsystem.updateHardware();
            turretSubsystem.updateHardware();
            double turretStartTicks = robot.turret.getCurrentPosition();

            multiTelemetry.addData("turret/start pos (ticks)", turretStartTicks);
            multiTelemetry.addData("turret/start pos (deg)", turretSubsystem.ticksToDegrees(turretStartTicks));
            multiTelemetry.addData("spindexer/start deg", spindexerTargetDeg);
            multiTelemetry.addData("spindexer/current deg", spindexerSubsystem.posDegrees);
            multiTelemetry.update();
        }

        waitForStart();

        spindexerTargetDeg = spindexerSubsystem.voltageToDegrees(robot.spindexerAnalog.getVoltage());
        spindexerSubsystem.setTargetDegrees(spindexerTargetDeg);
        spindexerSubsystem.updateHardware();
        turretSubsystem.updateHardware();

        while (opModeIsActive()) {
            updateTurretCoefficients();
            updateSpindexerCoefficients();

            turretSubsystem.setTarget(turretTargetDeg);
            spindexerSubsystem.setTargetDegrees(spindexerTargetDeg);

            turretSubsystem.updateHardware();
            spindexerSubsystem.updateHardware();

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
            multiTelemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("turretPosTicks", turretSubsystem.pos);
            packet.put("turretTargetTicks", turretSubsystem.target);
            packet.put("turretErrorTicks", turretError);
            packet.put("turretPosDeg", turretSubsystem.ticksToDegrees(turretSubsystem.pos));
            packet.put("spindexerDeg", spindexerSubsystem.posDegrees);
            packet.put("spindexerTargetDeg", spindexerTargetDeg);
            packet.put("spindexerErrorDeg", spindexerError);
            packet.put("spindexerVolts", spindexerSubsystem.posVoltage);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        robot.turret.setPower(0);
        robot.spindexer.setPower(0);
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
