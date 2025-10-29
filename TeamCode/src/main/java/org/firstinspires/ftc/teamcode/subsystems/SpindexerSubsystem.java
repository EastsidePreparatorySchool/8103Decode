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
    public double pos;
    public double target;
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
        pos = robot.spindexerAnalog.getVoltage();
        power = MathUtils.clamp(spindexerPIDF.calculate(pos, target), -1 ,1);
        robot.spindexer.setPower(power);
        robot.telemetry.addData("spindexer pos", pos);
        robot.telemetry.addData("spindexer target", target);
    }

    public void periodic() {
        switch(state) {
            case INTAKE_ONE:
                target = Common.SPINDEXER_INTAKE_ONE;
                break;
            case INTAKE_TWO:
                target = Common.SPINDEXER_INTAKE_TWO;
                break;
            case INTAKE_THREE:
                target = Common.SPINDEXER_INTAKE_THREE;
                break;
            case OUTTAKE_ONE:
                target = Common.SPINDEXER_OUTTAKE_ONE;
                break;
            case OUTTAKE_TWO:
                target = Common.SPINDEXER_OUTTAKE_TWO;
                break;
            case OUTTAKE_THREE:
                target = Common.SPINDEXER_OUTTAKE_THREE;
                break;
        }
        updateHardware();
    }
}