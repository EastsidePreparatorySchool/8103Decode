package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;

public class TurretSubsystem extends SubsystemBase {
    public enum TurretState {
        STOPPED,
        RUNNING
    }

    private RobotHardware robot;
    public volatile TurretState state;
    public double deg; // turret should ALWAYS START facing forward, positive is counterclockwise relative to forward and negative is clockwise relative to forward as viewed from the top of the robot
    public PIDFController turretPIDF;
    public int pos;
    public double target;
    public double power;
    public TurretSubsystem() {
        robot = RobotHardware.getInstance();
        setTurretState(TurretState.STOPPED);
        turretPIDF = new PIDFController(Common.TURRET_KP, Common.TURRET_KI, Common.TURRET_KD, Common.TURRET_KF);
    }

    public void setTurretState(TurretState turretState) {
        state = turretState;
    }

    public void setTarget(double degrees) {
        deg = normalizeDegrees(degrees);
        double ticksPerRev = getTicksPerTurretRev();
        target = wrapToRange(degreesToTicks(deg), ticksPerRev);
    }

    public boolean withinTolerance() {
        double ticksPerRev = getTicksPerTurretRev();
        double errorTicks = shortestError(target, pos, ticksPerRev);
        return Math.abs(errorTicks) < Common.TURRET_TOLERANCE_TICKS;
    }

    public void updateHardware() {
        pos = robot.turret.getCurrentPosition();
        double ticksPerRev = getTicksPerTurretRev();
        double errorTicks = shortestError(target, pos, ticksPerRev);
        double pidReference = pos + errorTicks;
        power = MathUtils.clamp(turretPIDF.calculate(pos, pidReference), -1, 1);
        robot.turret.setPower(power);
        robot.telemetry.addData("turret pos", pos);
        robot.telemetry.addData("turret pos (deg)", ticksToDegrees(pos));
        robot.telemetry.addData("turret target", target);
        robot.telemetry.addData("turret target (deg)", deg);
        robot.telemetry.addData("turret error (ticks)", errorTicks);
    }

    public void periodic() {
        if(state == TurretState.RUNNING) {
            updateHardware();
        }
    }

    public double degreesToTicks(double degrees) {
        return degrees * getTicksPerDegree();
    }

    public double ticksToDegrees(double ticks) {
        double ticksPerDegree = getTicksPerDegree();
        if (ticksPerDegree == 0) {
            return 0;
        }
        return ticks / ticksPerDegree;
    }

    private double normalizeDegrees(double degrees) {
        return wrapToRange(degrees, Common.TURRET_FULL_ROTATION_DEGREES);
    }

    private double wrapToRange(double value, double range) {
        if (range <= 0) {
            return value;
        }
        double wrapped = value % range;
        if (wrapped < 0) {
            wrapped += range;
        }
        return wrapped;
    }

    private double shortestError(double targetValue, double currentValue, double range) {
        if (range <= 0) {
            return targetValue - currentValue;
        }
        double halfRange = range / 2.0;
        double error = targetValue - currentValue;
        error = (error + halfRange) % range;
        if (error < 0) {
            error += range;
        }
        return error - halfRange;
    }

    private double getTicksPerTurretRev() {
        if (Common.TURRET_DRIVING_PULLEY_TEETH == 0) {
            return 0;
        }
        return Common.TURRET_TICKS_PER_MOTOR_REV * Common.TURRET_GEARBOX_RATIO * (Common.TURRET_DRIVEN_PULLEY_TEETH / Common.TURRET_DRIVING_PULLEY_TEETH);
    }

    private double getTicksPerDegree() {
        if (Common.TURRET_FULL_ROTATION_DEGREES == 0) {
            return 0;
        }
        return getTicksPerTurretRev() / Common.TURRET_FULL_ROTATION_DEGREES;
    }
}
