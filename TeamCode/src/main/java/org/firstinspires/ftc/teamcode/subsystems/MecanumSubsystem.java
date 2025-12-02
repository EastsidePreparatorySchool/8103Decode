package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;

public class MecanumSubsystem extends SubsystemBase {
    private final RobotHardware robot;
    private double lastFL;
    private double lastFR;
    private double lastBL;
    private double lastBR;

    public MecanumSubsystem() {
        robot = RobotHardware.getInstance();
    }

    public void setMotorPowers(double powerFL, double powerFR, double powerBL, double powerBR) {
        lastFL = powerFL;
        lastFR = powerFR;
        lastBL = powerBL;
        lastBR = powerBR;
        robot.powerMotors(powerFL, powerFR, powerBL, powerBR);
    }

    public void stop() {
        setMotorPowers(0.0, 0.0, 0.0, 0.0);
    }

    public double getLastFL() {
        return lastFL;
    }

    public double getLastFR() {
        return lastFR;
    }

    public double getLastBL() {
        return lastBL;
    }

    public double getLastBR() {
        return lastBR;
    }
}
