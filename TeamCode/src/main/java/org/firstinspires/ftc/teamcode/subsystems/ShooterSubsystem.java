package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;

public class ShooterSubsystem extends SubsystemBase {
    public enum ShooterState {
        ON,
        OFF
    }

    private RobotHardware robot;
    public volatile ShooterState state;
    public boolean update = false;
    public double pos;

    public ShooterSubsystem() {
        robot = RobotHardware.getInstance();
        setShooterState(ShooterState.OFF);
    }

    public void setShooterState (ShooterState shooterState) {
        state = shooterState;
        update = true;
    }

    public void tickServoPosition(int inc) {
        pos += inc;
        update = true;
    }

    public void updateHardware() {
        switch(state) {
            case ON:
                robot.flywheel.setPower(Common.FLYWHEEL_ON);
                break;
            case OFF:
                robot.flywheel.setPower(0);
                break;
        }
        robot.hood.setPosition(pos);
    }

    public void periodic() {
        if(update) {
            updateHardware();
            update = false;
        }
    }
}