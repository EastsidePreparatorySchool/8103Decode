package org.firstinspires.ftc.teamcode.subsystems;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.lib.Common;

public class SpindexerSubsystem extends SubsystemBase{
    public enum SpindexerState {
        SLOT1TRANSFER,
        SLOT1OPEN,
        SLOT2TRANSFER,
        SLOT2OPEN,
        SLOT3TRANSFER,
        SLOT3OPEN,
    }

    private RobotHardware robot;
    public boolean update = false;
    public volatile SpindexerState state;

    public SpindexerSubsystem() {
        robot = RobotHardware.getInstance();
    }

    public void setSpindexerState(SpindexerState spindexerState) {
        update = true;
        state = spindexerState;
    }

    public void updateHardware() {
        switch (state) {
            case SLOT1TRANSFER:
                robot.spindexer.setPosition(Common.SPINDEXER_SLOT1TRANSFER_DIRECTION);
                break;
            case SLOT1OPEN:
                robot.spindexer.setPosition(Common.SPINDEXER_SLOT1OPEN_DIRECTION);
            case SLOT2TRANSFER:
                robot.spindexer.setPosition(Common.SPINDEXER_SLOT2TRANSFER_DIRECTION);
                break;
            case SLOT2OPEN:
                robot.spindexer.setPosition(Common.SPINDEXER_SLOT2OPEN_DIRECTION);
            case SLOT3TRANSFER:
                robot.spindexer.setPosition(Common.SPINDEXER_SLOT3TRANSFER_DIRECTION);
                break;
            case SLOT3OPEN:
                robot.spindexer.setPosition(Common.SPINDEXER_SLOT3OPEN_DIRECTION);
            default:
                break;
        }
    }

    public void periodic() {
        if(update) {
            updateHardware();
            update = false;
        }
    }

}
