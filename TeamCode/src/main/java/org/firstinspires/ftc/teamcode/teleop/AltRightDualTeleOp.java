package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.lib.Common;

@TeleOp(name = "AltRightDualTeleOp", group = "Command")
public class AltRightDualTeleOp extends DualTeleOp {
    @Override
    public void initialize() {
        Common.SELECTED_FIELD_TARGET_X_IN = Common.ALT_RIGHT_FIELD_TARGET_X_IN;
        Common.SELECTED_FIELD_TARGET_Y_IN = Common.ALT_RIGHT_FIELD_TARGET_Y_IN;
        super.initialize();
    }
}

