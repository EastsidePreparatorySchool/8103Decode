package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.lib.AutoPoses;
import org.firstinspires.ftc.teamcode.lib.Common;

@Autonomous(name = "FarNinePieceAutoLeft", group = "Command")
public class FarNinePieceAutoLeft extends FarNinePieceAuto {
    @Override
    public void initialize() {
        Common.TARGET_X_IN = Common.LEFT_FIELD_TARGET_X_IN;
        Common.TARGET_Y_IN = Common.LEFT_FIELD_TARGET_Y_IN;
        Common.START_X_IN = Common.LEFT_FIELD_START_X_IN;
        Common.START_Y_IN = Common.LEFT_FIELD_START_Y_IN;
        Common.ACTUAL_TARGET_X_IN = Common.LEFT_FIELD_ACTUAL_TARGET_X_IN;
        Common.ACTUAL_TARGET_Y_IN = Common.LEFT_FIELD_ACTUAL_TARGET_Y_IN;

        AutoPoses.START = AutoPoses.LEFT_START;
        AutoPoses.BALL1_PICKUP = AutoPoses.LEFT_BALL1_PICKUP;
        AutoPoses.BALL1_PICKUP_CONTROL1 = AutoPoses.LEFT_BALL1_PICKUP_CONTROL1;
        AutoPoses.BALL2_PICKUP = AutoPoses.LEFT_BALL2_PICKUP;
        AutoPoses.BALL3_PICKUP = AutoPoses.LEFT_BALL3_PICKUP;
        AutoPoses.SHOT1 = AutoPoses.LEFT_SHOT1_FAR;
        AutoPoses.BALL4_PICKUP = AutoPoses.LEFT_BALL4_PICKUP;
        AutoPoses.BALL4_PICKUP_CONTROL1 = AutoPoses.LEFT_BALL4_PICKUP_CONTROL1_FAR;
        AutoPoses.BALL5_PICKUP = AutoPoses.LEFT_BALL5_PICKUP;
        AutoPoses.BALL6_PICKUP = AutoPoses.LEFT_BALL6_PICKUP;
        AutoPoses.SHOT2 = AutoPoses.LEFT_SHOT2_FAR;

        super.initialize();
    }
}
