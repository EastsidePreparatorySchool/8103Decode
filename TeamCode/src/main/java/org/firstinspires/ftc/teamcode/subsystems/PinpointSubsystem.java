package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class  PinpointSubsystem extends SubsystemBase {
    private final RobotHardware robot;
    private Pose2D pose = new Pose2D(DistanceUnit.INCH, 0.0, 0.0, AngleUnit.DEGREES, 0.0);
    private double defaultXInches = 0.0;
    private double defaultYInches = 0.0;
    private double defaultHeadingDeg = 0.0;

    public PinpointSubsystem() {
        robot = RobotHardware.getInstance();
    }

    public void initializePose(double xInches, double yInches, double headingDegrees) {
        Pose2D startPose = new Pose2D(DistanceUnit.INCH, xInches, yInches, AngleUnit.DEGREES, headingDegrees);
        robot.pinpoint.setPosition(startPose);
        pose = startPose;
        defaultXInches = xInches;
        defaultYInches = yInches;
        defaultHeadingDeg = headingDegrees;
    }

    public Pose2D getPose() {
        return pose;
    }

    public double getHeadingDegrees() {
        return pose.getHeading(AngleUnit.DEGREES);
    }

    public double getHeadingRadians() {
        return pose.getHeading(AngleUnit.RADIANS);
    }

    public double getXInches() {
        return pose.getX(DistanceUnit.INCH);
    }

    public double getYInches() {
        return pose.getY(DistanceUnit.INCH);
    }

    @Override
    public void periodic() {
        robot.pinpoint.update();
        pose = robot.pinpoint.getPosition();
        robot.robotX = pose.getX(DistanceUnit.INCH);
        robot.robotY = pose.getY(DistanceUnit.INCH);
        robot.robotHeadingDeg = pose.getHeading(AngleUnit.DEGREES);
    }

    // Velocity getters (converted from Pinpoint mm/s to in/s)
    public double getVelXInPerSec() {
        return robot.pinpoint.getVelX() / 25.4;  // mm to inches
    }

    public double getVelYInPerSec() {
        return robot.pinpoint.getVelY() / 25.4;
    }

    public double getVelHeadingRadPerSec() {
        return robot.pinpoint.getHeadingVelocity();  
    }
    public void resetToConfiguredStart() {
        initializePose(defaultXInches, defaultYInches, defaultHeadingDeg);
    }
}
