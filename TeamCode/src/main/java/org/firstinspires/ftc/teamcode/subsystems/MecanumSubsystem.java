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

    public void drive(double forward, double strafe, double turn, double multiplier) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);
        lastFL = multiplier * (forward + strafe + turn) / denominator;
        lastFR = multiplier * (forward - strafe - turn) / denominator;
        lastBL = multiplier * (forward - strafe + turn) / denominator;
        lastBR = multiplier * (forward + strafe - turn) / denominator;
        robot.powerMotors(lastFL, lastFR, lastBL, lastBR);
    }

    public void stop() {
        drive(0.0, 0.0, 0.0, 0.0);
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
