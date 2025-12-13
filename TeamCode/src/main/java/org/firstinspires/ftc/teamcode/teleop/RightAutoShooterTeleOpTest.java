package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Common;

@Disabled
@TeleOp(name = "RightDualTeleOp", group = "Testing")
public class RightAutoShooterTeleOpTest extends AutoShooterTeleOpTest {
    @Override
    public void initialize() {
        Common.TARGET_X_IN = Common.RIGHT_FIELD_TARGET_X_IN;
        Common.TARGET_Y_IN = Common.RIGHT_FIELD_TARGET_Y_IN;
        Common.START_X_IN = Common.RIGHT_FAR_START_X_IN;
        Common.START_Y_IN = Common.RIGHT_FAR_START_Y_IN;
        Common.ACTUAL_TARGET_X_IN = Common.RIGHT_FIELD_ACTUAL_TARGET_X_IN;
        Common.ACTUAL_TARGET_Y_IN = Common.RIGHT_FIELD_ACTUAL_TARGET_Y_IN;
        super.initialize();
    }
}
