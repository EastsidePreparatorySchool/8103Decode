package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;

public class DistanceSensorSubsystem extends SubsystemBase {
    public enum DistanceSensorState {
        OFF,
        ON
    }

    private final RobotHardware robot;
    public volatile DistanceSensorState state;
    public boolean within50MM;
    public DistanceSensorSubsystem() {
        robot = RobotHardware.getInstance();
        state = DistanceSensorState.OFF;
    }

    public void setState(DistanceSensorState newState) {
        state = newState;
    }

    public boolean getWithin50MM() {
        return within50MM;
    }

    private void readDistance() {
        within50MM = robot.distanceSensor.getState();
    }

    @Override
    public void periodic() {
        switch(state) {
            case OFF:
                break;
            case ON:
                readDistance();
        }
    }
}
