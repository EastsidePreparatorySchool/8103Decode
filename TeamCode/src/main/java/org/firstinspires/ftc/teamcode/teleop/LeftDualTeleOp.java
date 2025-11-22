package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.lib.Common;

@TeleOp(name = "LeftDualTeleOp", group = "Command")
public class LeftDualTeleOp extends DualTeleOp {
    @Override
    public void initialize() {
        Common.TARGET_X_IN = Common.LEFT_FIELD_TARGET_X_IN;
        Common.TARGET_Y_IN = Common.LEFT_FIELD_TARGET_Y_IN;
        Common.START_X_IN = Common.LEFT_FIELD_START_X_IN;
        Common.START_Y_IN = Common.LEFT_FIELD_START_Y_IN;
        Common.ACTUAL_TARGET_X_IN = Common.LEFT_FIELD_ACTUAL_TARGET_X_IN;
        Common.ACTUAL_TARGET_Y_IN = Common.LEFT_FIELD_ACTUAL_TARGET_Y_IN;
        super.initialize();
    }
}

