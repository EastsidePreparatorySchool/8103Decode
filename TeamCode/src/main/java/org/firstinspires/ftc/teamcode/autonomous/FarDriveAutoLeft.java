package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.lib.AutoPoses;
import org.firstinspires.ftc.teamcode.lib.Common;

@Autonomous(name = "FarDriveAutoLeft", group = "Command")
public class FarDriveAutoLeft extends FarDriveAuto {
    @Override
    public void initialize() {
        Common.TARGET_X_IN = Common.LEFT_FIELD_TARGET_X_IN;
        Common.TARGET_Y_IN = Common.LEFT_FIELD_TARGET_Y_IN;
        Common.START_X_IN = Common.LEFT_FIELD_START_X_IN;
        Common.START_Y_IN = Common.LEFT_FIELD_START_Y_IN;
        Common.ACTUAL_TARGET_X_IN = Common.LEFT_FIELD_ACTUAL_TARGET_X_IN;
        Common.ACTUAL_TARGET_Y_IN = Common.LEFT_FIELD_ACTUAL_TARGET_Y_IN;

        AutoPoses.START = AutoPoses.LEFT_START;
        
        driveOffset = -20;

        super.initialize();
    }
}
