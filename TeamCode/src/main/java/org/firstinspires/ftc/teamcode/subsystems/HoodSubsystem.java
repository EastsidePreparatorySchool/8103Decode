package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;

public class HoodSubsystem extends SubsystemBase {
    private final RobotHardware robot;
    public double hoodPos;

    public HoodSubsystem() {
        robot = RobotHardware.getInstance();
        hoodPos = Common.HOOD_INITIAL_POS;
    }

    public void setHoodPosition(double pos) {
        hoodPos = pos;
    }

    public void tickServoPosition(int inc) {
        hoodPos += inc;
    }

    public void updateHardware() {
        robot.hood.setPosition(hoodPos);
    }

    @Override
    public void periodic() {
        updateHardware();
    }
}

