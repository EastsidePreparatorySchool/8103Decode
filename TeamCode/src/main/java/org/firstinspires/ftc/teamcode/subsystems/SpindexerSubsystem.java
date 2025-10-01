package org.firstinspires.ftc.teamcode.subsystems;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.lib.Common;

public class SpindexerSubsystem extends SubsystemBase{
    public enum SpindexerState {
        SLOT1,
        SLOT2,
        SLOT3,
        //COUNTERCLOCKWISE,
        //CLOCKWISE,
        //STOPPED
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
            case SLOT1:
                robot.spindexer.setPosition(Common.SPINDEXER_SLOT1_DIRECTION);
                break;
            case SLOT2:
                robot.spindexer.setPosition(Common.SPINDEXER_SLOT2_DIRECTION);
                break;
            case SLOT3:
                robot.spindexer.setPosition(Common.SPINDEXER_SLOT3_DIRECTION);
            //case COUNTERCLOCKWISE:
                //robot.spindexer.setPower(Common.SPINDEXER_COUNTERCLOCKWISE_POWER);
            //case CLOCKWISE:
                //robot.spindexer.setPower(Common.SPINDEXER_CLOCKWISE_POWER);
            //case STOPPED:
                //robot.spindexer.setPower(Common.SPINDEXER_NO_POWER);
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
