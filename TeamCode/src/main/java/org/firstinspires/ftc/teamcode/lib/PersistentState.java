package org.firstinspires.ftc.teamcode.lib;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

    public static void saveFromRobot() {
        RobotHardware robot = RobotHardware.getInstance();
        if (robot.pinpointSubsystem != null) {
            savedXInches = robot.pinpointSubsystem.getPose().getX(DistanceUnit.INCH);
            savedYInches = robot.pinpointSubsystem.getPose().getY(DistanceUnit.INCH);
            savedHeadingDeg = robot.pinpointSubsystem.getPose().getHeading(AngleUnit.DEGREES);
            hasSavedPose = true;
        }
        if (robot.turretSubsystem != null) {
            // Use the target degrees field for continuity
            savedTurretDegrees = robot.turretSubsystem.deg;
            hasSavedTurret = true;
        }
    }
}

