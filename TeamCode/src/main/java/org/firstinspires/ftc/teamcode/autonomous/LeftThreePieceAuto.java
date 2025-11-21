package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lib.Common;

@Autonomous(name = "LeftThreePieceAuto", group = "Command")
public class LeftThreePieceAuto extends ThreePieceAuto {
    @Override
    public void initialize() {
        Common.TARGET_X_IN = Common.LEFT_FIELD_TARGET_X_IN;
        Common.TARGET_Y_IN = Common.LEFT_FIELD_TARGET_Y_IN;
        super.initialize();
    }
}
