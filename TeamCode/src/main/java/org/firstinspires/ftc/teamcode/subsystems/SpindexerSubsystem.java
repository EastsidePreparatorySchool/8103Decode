package org.firstinspires.ftc.teamcode.subsystems;
import com.seattlesolvers.solverslib.command.SubsystemBase;

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

    private final RobotHardware robot;
    public volatile SpindexerState state;
    public double targetPosition; // servo position [0,1]

    public SpindexerSubsystem() {
        robot = RobotHardware.getInstance();
        setSpindexerState(SpindexerState.INTAKE_ONE);
    }

    public void setSpindexerState(SpindexerState spindexerState) {
        state = spindexerState;
    }

    public void updateHardware() {
        robot.spindexer.setPosition(targetPosition);
    }

    public void periodic() {
        switch(state) {
            case INTAKE_ONE:
                targetPosition = Common.SPINDEXER_INTAKE_ONE;
                break;
            case INTAKE_TWO:
                targetPosition = Common.SPINDEXER_INTAKE_TWO;
                break;
            case INTAKE_THREE:
                targetPosition = Common.SPINDEXER_INTAKE_THREE;
                break;
            case OUTTAKE_ONE:
                targetPosition = Common.SPINDEXER_OUTTAKE_ONE;
                break;
            case OUTTAKE_TWO:
                targetPosition = Common.SPINDEXER_OUTTAKE_TWO;
                break;
            case OUTTAKE_THREE:
                targetPosition = Common.SPINDEXER_OUTTAKE_THREE;
                break;
        }
        updateHardware();
    }

    public void setTargetPosition(double position) {
        if (position < 0.0) position = 0.0;
        if (position > 1.0) position = 1.0;
        targetPosition = position;
    }

    public double getPosition() {
        return robot.spindexer.getPosition();
    }
}
