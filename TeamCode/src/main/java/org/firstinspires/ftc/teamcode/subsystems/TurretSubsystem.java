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
    public double deg; // positive is counterclockwise relative to forward, may range beyond +/-360 to describe unwrapped orientation
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
        if (turretState == TurretState.STOPPED) {
            robot.turret.setPower(0.0);
        }
    }

    public void setTarget(double degrees) {
        double safeDegrees = selectSafeTargetDegrees(degrees);
        deg = safeDegrees;
        double ticksPerRev = getTicksPerTurretRev();
        double normalizedDegrees = normalizeDegrees(safeDegrees);
        target = wrapToRange(degreesToTicks(normalizedDegrees), ticksPerRev);
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

    private double selectSafeTargetDegrees(double desiredDegrees) {
        double limit = Math.abs(Common.TURRET_WIRE_WRAP_LIMIT_DEGREES);
        if (limit <= 0) {
            return desiredDegrees;
        }

        double currentDegrees = ticksToDegrees(robot.turret.getCurrentPosition());
        double fullRotation = Common.TURRET_FULL_ROTATION_DEGREES;
        if (fullRotation <= 0) {
            return Math.max(-limit, Math.min(limit, desiredDegrees));
        }

        double bestCandidate = desiredDegrees;
        double bestCost = Double.POSITIVE_INFINITY;
        int maxTurns = (int) Math.ceil((2.0 * limit) / fullRotation) + 1;

        for (int offset = -maxTurns; offset <= maxTurns; offset++) {
            double candidate = desiredDegrees + offset * fullRotation;
            if (Math.abs(candidate) > limit) {
                continue;
            }
            double cost = Math.abs(candidate - currentDegrees);
            if (cost < bestCost) {
                bestCost = cost;
                bestCandidate = candidate;
            }
        }

        if (bestCost == Double.POSITIVE_INFINITY) {
            return Math.max(-limit, Math.min(limit, desiredDegrees));
        }

        return bestCandidate;
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
