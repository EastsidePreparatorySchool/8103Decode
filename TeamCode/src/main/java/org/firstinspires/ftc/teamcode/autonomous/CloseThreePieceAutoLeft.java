package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.lib.Common;

@Autonomous(name = "CloseThreePieceAutoLeft", group = "Command")
public class CloseThreePieceAutoLeft extends CloseThreePieceAuto {
    @Override
    public void initialize() {
        Common.START_X_IN = Common.CLOSE_LEFT_FIELD_START_X_IN;
        Common.START_Y_IN = Common.CLOSE_LEFT_FIELD_START_Y_IN;
        Common.START_HEADING_DEG = Common.CLOSE_LEFT_FIELD_START_HEADING_DEG;
        
        Common.TARGET_X_IN = Common.LEFT_FIELD_TARGET_X_IN;
        Common.TARGET_Y_IN = Common.LEFT_FIELD_TARGET_Y_IN;
        Common.ACTUAL_TARGET_X_IN = Common.LEFT_FIELD_ACTUAL_TARGET_X_IN;
        Common.ACTUAL_TARGET_Y_IN = Common.LEFT_FIELD_ACTUAL_TARGET_Y_IN;

        super.initialize();
    }
}
