package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;
//import org.firstinspires.ftc.teamcode.lib.Common;

public class TransferSubsystem extends SubsystemBase{
    public enum TransferState {
        UP,
        DOWN
    }

    private RobotHardware robot;
    public volatile TransferState transferState;

    public boolean locked = false;
    public boolean update = false;

    public TransferSubsystem() {
        robot = RobotHardware.getInstance();
        setTransferState(TransferState.DOWN);
    }

    public void setTransferState(TransferState transferState) {
        this.transferState = transferState;
        update = true;
    }

    public void toggleTransferState() {
        update = true;
        switch (transferState) {
            case UP:
//                robot.transfer.setPosition(Common.TRANSFER_UP);
                break;
            case DOWN:
//                robot.transfer.setPosition(Common.TRANSFER_DOWN);
                break;
        }
    }

    public void updateHardware() {
        switch(transferState) {
            case UP:
//                robot.transfer.setPosition(Common.TRANSFER_UP);
                break;
            case DOWN:
//                robot.transfer.setPosition(Common.TRANSFER_DOWN);
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