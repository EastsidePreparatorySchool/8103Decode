package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.lib.AutoPoses;
import org.firstinspires.ftc.teamcode.lib.Common;

@Autonomous(name = "CloseNinePieceAutoLeft", group = "Command")
public class CloseNinePieceAutoLeft extends CloseNinePieceAuto {
    @Override
    public void initialize() {
        Common.TARGET_X_IN = Common.LEFT_FIELD_TARGET_X_IN;
        Common.TARGET_Y_IN = Common.LEFT_FIELD_TARGET_Y_IN;
        Common.START_X_IN = Common.CLOSE_LEFT_FIELD_START_X_IN;
        Common.START_Y_IN = Common.CLOSE_LEFT_FIELD_START_Y_IN;
        Common.START_HEADING_DEG = 0;
        Common.ACTUAL_TARGET_X_IN = Common.LEFT_FIELD_ACTUAL_TARGET_X_IN;
        Common.ACTUAL_TARGET_Y_IN = Common.LEFT_FIELD_ACTUAL_TARGET_Y_IN;

        AutoPoses.START = AutoPoses.LEFT_START;
        AutoPoses.SHOT_CLOSE = AutoPoses.LEFT_SHOT_CLOSE;
        AutoPoses.BALL_7_PICKUP = AutoPoses.LEFT_BALL_7_PICKUP;
        AutoPoses.BALL_7_PICKUP_CONTROL1 = AutoPoses.LEFT_BALL_7_PICKUP_CONTROL1_CLOSE;
        AutoPoses.BALL_8_PICKUP = AutoPoses.LEFT_BALL_8_PICKUP;
        AutoPoses.BALL_9_PICKUP = AutoPoses.LEFT_BALL_9_PICKUP;
        AutoPoses.BALL4_PICKUP = AutoPoses.LEFT_BALL4_PICKUP;
        AutoPoses.BALL4_PICKUP_CONTROL1 = AutoPoses.LEFT_BALL4_PICKUP_CONTROL1_CLOSE;
        AutoPoses.BALL5_PICKUP = AutoPoses.LEFT_BALL5_PICKUP;
        AutoPoses.BALL6_PICKUP = AutoPoses.LEFT_BALL6_PICKUP;

        super.initialize();
    }
}
