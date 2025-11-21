package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.lib.Common;

@TeleOp(name = "RightTeleOp", group = "Command")
public class RightTeleOp extends SoloTeleOp {
    @Override
    public void initialize() {
        Common.TARGET_X_IN = Common.RIGHT_FIELD_TARGET_X_IN;
        Common.TARGET_Y_IN = Common.RIGHT_FIELD_TARGET_Y_IN;
        super.initialize();
    }
}
