package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;

public class TransferSubsystem extends SubsystemBase {
    public enum TransferState {
        DOWN,
        UP
    }

    private RobotHardware robot;
    public volatile TransferState state;
    public boolean update;
    public TransferSubsystem() {
        robot = RobotHardware.getInstance();
        setTransferState(TransferState.DOWN);
        update = true;
    }

    public void setTransferState(TransferState transferState) {
        state = transferState;
        update = true;
    }

    public void updateHardware() {
        switch(state) {
            case DOWN:
                robot.transfer.setPosition(Common.TRANSFER_DOWN);
                break;
            case UP:
                robot.transfer.setPosition(Common.TRANSFER_UP);
                break;
        }
    }

    public void periodic() {
        if(update) {
            updateHardware();
        }
    }
}
