package org.firstinspires.ftc.teamcode.subsystems;

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
    public double kf;
    public double turretX;
    public double turretY;
    private double encoderOffset = 0.0;
    public TurretSubsystem() {
        robot = RobotHardware.getInstance();
        setTurretState(TurretState.STOPPED);
        turretPIDF = new PIDFController(Common.TURRET_KP, Common.TURRET_KI, Common.TURRET_KD, 0);
        kf = Common.TURRET_KF;
    }

    public void setTurretState(TurretState turretState) {
        state = turretState;
        if (turretState == TurretState.STOPPED) {
            robot.turret.setPower(0.0);
        }
    }

    public void setTarget(double degrees) {
        double safeDegrees = selectSafeTargetDegrees(degrees);
        deg = safeDegrees; // keep unwrapped target for telemetry
        // Use absolute (unwrapped) ticks so we can choose direction that respects wire wrap limit
        target = degreesToTicks(safeDegrees);
    }

    public void setTurretAngle(double degrees) {
        // We want the current position to be read as 'degrees'
        // currentPos = rawPos + offset
        // degreesToTicks(degrees) = rawPos + offset
        // offset = degreesToTicks(degrees) - rawPos
        double currentRawTicks = robot.turret.getCurrentPosition();
        encoderOffset = degreesToTicks(degrees) - currentRawTicks;
        
        // Update target and deg to match so we don't jump
        deg = degrees;
        target = degreesToTicks(degrees);
    }

    public boolean withinTolerance() {
        double errorTicks = target - pos;
        return ticksToDegrees(Math.abs(errorTicks)) < Common.TURRET_TOLERANCE_DEG;
    }

    public void updateHardware() {
        pos = (int) (robot.turret.getCurrentPosition() + encoderOffset);
        power = MathUtils.clamp(turretPIDF.calculate(pos, target), -1, 1);
        if(!withinTolerance()) {
            power += kf * Math.signum(turretPIDF.getPositionError());
        }
        double vbat = robot.getBatteryVoltage();
        if (vbat > 1e-3) {
            double scale = Common.NOMINAL_BATTERY_VOLTAGE / vbat;
            power *= scale;
        }
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
        return Common.TURRET_TICKS_PER_MOTOR_REV * Common.TURRET_GEARBOX_RATIO * (Common.TURRET_DRIVEN_PULLEY_TEETH / Common.TURRET_DRIVING_PULLEY_TEETH);
    }

    private double getTicksPerDegree() {
        return getTicksPerTurretRev() / Common.TURRET_FULL_ROTATION_DEGREES;
    }
}
