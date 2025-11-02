package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.lib.Common;

@TeleOp(name = "AltLeftDualTeleOp", group = "Command")
public class AltLeftDualTeleOp extends DualTeleOp {
    @Override
    public void initialize() {
        Common.SELECTED_FIELD_TARGET_X_IN = Common.ALT_LEFT_FIELD_TARGET_X_IN;
        Common.SELECTED_FIELD_TARGET_Y_IN = Common.ALT_LEFT_FIELD_TARGET_Y_IN;
        super.initialize();
    }
}

