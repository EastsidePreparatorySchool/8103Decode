package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.lib.AutoPoses;
import org.firstinspires.ftc.teamcode.lib.Common;

@Autonomous(name = "FarNinePieceAutoRight", group = "Command")
public class FarNinePieceAutoRight extends FarNinePieceAuto {
    @Override
    public void initialize() {
        Common.TARGET_X_IN = Common.RIGHT_FIELD_TARGET_X_IN;
        Common.TARGET_Y_IN = Common.RIGHT_FIELD_TARGET_Y_IN;
        Common.START_X_IN = Common.RIGHT_FIELD_START_X_IN;
        Common.START_Y_IN = Common.RIGHT_FIELD_START_Y_IN;
        Common.ACTUAL_TARGET_X_IN = Common.RIGHT_FIELD_ACTUAL_TARGET_X_IN;
        Common.ACTUAL_TARGET_Y_IN = Common.RIGHT_FIELD_ACTUAL_TARGET_Y_IN;

        AutoPoses.START = AutoPoses.RIGHT_START;
        AutoPoses.BALL1_PICKUP = AutoPoses.RIGHT_BALL1_PICKUP;
        AutoPoses.BALL1_PICKUP_CONTROL1 = AutoPoses.RIGHT_BALL1_PICKUP_CONTROL1;
        AutoPoses.BALL2_PICKUP = AutoPoses.RIGHT_BALL2_PICKUP;
        AutoPoses.BALL3_PICKUP = AutoPoses.RIGHT_BALL3_PICKUP;
        AutoPoses.SHOT1 = AutoPoses.RIGHT_SHOT1;
        AutoPoses.BALL4_PICKUP = AutoPoses.RIGHT_BALL4_PICKUP;
        AutoPoses.BALL4_PICKUP_CONTROL1 = AutoPoses.RIGHT_BALL4_PICKUP_CONTROL1;
        AutoPoses.BALL5_PICKUP = AutoPoses.RIGHT_BALL5_PICKUP;
        AutoPoses.BALL6_PICKUP = AutoPoses.RIGHT_BALL6_PICKUP;
        AutoPoses.SHOT2 = AutoPoses.RIGHT_SHOT2;

        super.initialize();
    }
}
