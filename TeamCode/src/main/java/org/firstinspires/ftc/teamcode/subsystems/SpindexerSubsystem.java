package org.firstinspires.ftc.teamcode.subsystems;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;

public class SpindexerSubsystem extends SubsystemBase {
    public enum SpindexerState {
        INTAKE_ONE,
        INTAKE_TWO,
        INTAKE_THREE,
        OUTTAKE_ONE,
        OUTTAKE_TWO,
        OUTTAKE_THREE
    }

    private RobotHardware robot;
    public volatile SpindexerState state;
    public PIDFController spindexerPIDF;
    public double posVoltage;
    public double posDegrees;
    public double targetDegrees;
    public double power;
    public SpindexerSubsystem() {
        robot = RobotHardware.getInstance();
        setSpindexerState(SpindexerState.INTAKE_ONE);
        spindexerPIDF = new PIDFController(Common.SPINDEXER_KP, Common.SPINDEXER_KI, Common.SPINDEXER_KD, Common.SPINDEXER_KF);
    }

    public void setSpindexerState(SpindexerState spindexerState) {
        state = spindexerState;
    }

    public void updateHardware() {
        posVoltage = robot.spindexerAnalog.getVoltage();
        posDegrees = wrapToRange(voltageToDegrees(posVoltage), Common.AXON_DEGREE_RANGE);
        double wrappedTarget = wrapToRange(targetDegrees, Common.AXON_DEGREE_RANGE);
        double error = shortestError(wrappedTarget, posDegrees, Common.AXON_DEGREE_RANGE);
        double pidReference = posDegrees + error;
        power = MathUtils.clamp(spindexerPIDF.calculate(posDegrees, pidReference), -1 ,1);
        robot.spindexer.setPower(power);
        robot.telemetry.addData("spindexer pos (deg)", posDegrees);
        robot.telemetry.addData("spindexer pos (volts)", posVoltage);
        robot.telemetry.addData("spindexer target (deg)", wrappedTarget);
        robot.telemetry.addData("spindexer error (deg)", error);
    }

    public void periodic() {
        switch(state) {
            case INTAKE_ONE:
                targetDegrees = Common.SPINDEXER_INTAKE_ONE;
                break;
            case INTAKE_TWO:
                targetDegrees = Common.SPINDEXER_INTAKE_TWO;
                break;
            case INTAKE_THREE:
                targetDegrees = Common.SPINDEXER_INTAKE_THREE;
                break;
            case OUTTAKE_ONE:
                targetDegrees = Common.SPINDEXER_OUTTAKE_ONE;
                break;
            case OUTTAKE_TWO:
                targetDegrees = Common.SPINDEXER_OUTTAKE_TWO;
                break;
            case OUTTAKE_THREE:
                targetDegrees = Common.SPINDEXER_OUTTAKE_THREE;
                break;
        }
        targetDegrees = wrapToRange(targetDegrees, Common.AXON_DEGREE_RANGE);
        updateHardware();
    }

    public void setTargetDegrees(double degrees) {
        targetDegrees = wrapToRange(degrees, Common.AXON_DEGREE_RANGE);
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

    public double voltageToDegrees(double voltage) {
        return (voltage / Common.AXON_ANALOG_RANGE) * Common.AXON_DEGREE_RANGE;
    }

    public double degreesToVoltage(double degrees) {
        double wrappedDegrees = wrapToRange(degrees, Common.AXON_DEGREE_RANGE);
        return (wrappedDegrees / Common.AXON_DEGREE_RANGE) * Common.AXON_ANALOG_RANGE;
    }
}
