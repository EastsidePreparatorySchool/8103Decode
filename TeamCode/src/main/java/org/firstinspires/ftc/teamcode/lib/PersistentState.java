package org.firstinspires.ftc.teamcode.lib;

/**
 * Simple static holder for persisting pose and turret state across opmodes.
 */
public class PersistentState {
    public static boolean hasSavedPose = false;
    public static double savedXInches = 0.0;
    public static double savedYInches = 0.0;
    public static double savedHeadingDeg = 0.0;

    public static boolean hasSavedTurret = false;
    public static double savedTurretDegrees = 0.0;

    // Ball pattern persistence (detected via Limelight AprilTags)
    public static boolean hasSavedBallPattern = false;
    public static Common.BallPattern savedBallPattern = Common.BallPattern.UNKNOWN;

    public static void saveFromRobot() {
        RobotHardware robot = RobotHardware.getInstance();
            savedXInches = robot.robotX;
            savedYInches = robot.robotY;
            savedHeadingDeg = robot.robotHeadingDeg;
            hasSavedPose = true;
        if (robot.turretSubsystem != null) {
            // Use the target degrees field for continuity
            savedTurretDegrees = robot.turretSubsystem.deg;
            hasSavedTurret = true;
        }
    }

    public static void saveBallPattern(Common.BallPattern pattern) {
        savedBallPattern = pattern;
        hasSavedBallPattern = true;
    }

    public static void clearBallPattern() {
        hasSavedBallPattern = false;
        savedBallPattern = Common.BallPattern.UNKNOWN;
    }
}

