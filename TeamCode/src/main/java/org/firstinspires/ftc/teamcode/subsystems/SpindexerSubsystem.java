package org.firstinspires.ftc.teamcode.subsystems;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.lib.Common;

public class SpindexerSubsystem extends SubsystemBase{
    public enum SpindexerState {
        SLOT1,
        SLOT2,
        SLOT3,
        COUNTERCLOCKWISE,
        CLOCKWISE,
        STOPPED
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
                robot.spindexer.setPosition(Common.CLAW_SERVO_CLOSED);
                break;
            case SLOT2:
                robot.spindexer.setPosition(Common.CLAW_SERVO_OPEN);
                break;
            case SLOT3:
                robot.spindexer.setPosition(Common.CLAW_SERVO_WIDE);
            case COUNTERCLOCKWISE:
                robot.spindexer.setPower(-1.0);
            case CLOCKWISE:
                robot.spindexer.setPower(1.0);
            case STOPPED:
                robot.spindexer.setPower(0);
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
