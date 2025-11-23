package org.firstinspires.ftc.teamcode.lib;

import com.pedropathing.geometry.Pose;

public class AutoPoses {
    public static Pose RIGHT_START = new Pose(Common.RIGHT_FIELD_START_X_IN, Common.RIGHT_FIELD_START_Y_IN, Common.START_HEADING_DEG);
    public static Pose RIGHT_BALL1_PICKUP = new Pose(106.5, 35, 0);
    public static Pose RIGHT_BALL1_PICKUP_CONTROL1 = new Pose(87, 35);
    public static Pose RIGHT_BALL2_PICKUP = new Pose(111.5, 35);
    public static Pose RIGHT_BALL3_PICKUP = new Pose(116.5, 35);
    public static Pose RIGHT_SHOT1 = new Pose(80, 18, Math.toRadians(270));
    public static Pose RIGHT_BALL4_PICKUP = new Pose(106.5, 59, 0);
    public static Pose RIGHT_BALL4_PICKUP_CONTROL1 = new Pose(80, 59);
    public static Pose RIGHT_BALL5_PICKUP = new Pose(111.5, 59);
    public static Pose RIGHT_BALL6_PICKUP = new Pose(116.5, 59);
    public static Pose RIGHT_SHOT2 = new Pose(80, 18, Math.toRadians(270));
    public static Pose LEFT_START = new Pose(Common.LEFT_FIELD_START_X_IN, Common.LEFT_FIELD_START_Y_IN, Common.START_HEADING_DEG);
    public static Pose LEFT_BALL1_PICKUP = new Pose(144 - 106.5, 35, Math.toRadians(180));
    public static Pose LEFT_BALL1_PICKUP_CONTROL1 = new Pose(144 - 87, 35);
    public static Pose LEFT_BALL2_PICKUP = new Pose(144 - 111.5, 35);
    public static Pose LEFT_BALL3_PICKUP = new Pose(144 - 116.6, 35);
    public static Pose LEFT_SHOT1 = new Pose(144 - 80, 18, Math.toRadians(270));
    public static Pose LEFT_BALL4_PICKUP = new Pose(144 - 106.5, 59, Math.toRadians(180));
    public static Pose LEFT_BALL4_PICKUP_CONTROL1 = new Pose(144 - 80, 59);
    public static Pose LEFT_BALL5_PICKUP = new Pose(144 - 111.5, 59);
    public static Pose LEFT_BALL6_PICKUP = new Pose(144 - 116.5, 59);
    public static Pose LEFT_SHOT2 = new Pose(144 - 80, 18, Math.toRadians(270));
    public static Pose START = RIGHT_START;
    public static Pose BALL1_PICKUP = RIGHT_BALL1_PICKUP;
    public static Pose BALL1_PICKUP_CONTROL1 = RIGHT_BALL1_PICKUP_CONTROL1;
    public static Pose BALL2_PICKUP = RIGHT_BALL2_PICKUP;
    public static Pose BALL3_PICKUP = RIGHT_BALL3_PICKUP;
    public static Pose SHOT1 = RIGHT_SHOT1;
    public static Pose BALL4_PICKUP = RIGHT_BALL4_PICKUP;
    public static Pose BALL4_PICKUP_CONTROL1 = RIGHT_BALL4_PICKUP_CONTROL1;
    public static Pose BALL5_PICKUP = RIGHT_BALL5_PICKUP;
    public static Pose BALL6_PICKUP = RIGHT_BALL6_PICKUP;
    public static Pose SHOT2 = RIGHT_SHOT2;
}
