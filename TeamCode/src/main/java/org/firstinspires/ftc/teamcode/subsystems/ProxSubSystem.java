package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;

public class ProxSubsystem extends SubsystemBase {
    private String port;

    public ProxSubsystem() {
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
