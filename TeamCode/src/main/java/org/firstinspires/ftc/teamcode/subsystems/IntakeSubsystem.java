package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;

public class IntakeSubsystem extends SubsystemBase {
    public enum IntakeState {
        FORWARD,
        BACKWARD,
        STOPPED
    }

    private RobotHardware robot;
    public volatile IntakeState state;
    public boolean update = false;

    public IntakeSubsystem() {
        robot = RobotHardware.getInstance();
        setIntakeState(IntakeState.STOPPED);
    }

    public void setIntakeState(IntakeState intakeState) {
        state = intakeState;
        update = true;
    }

    public void updateHardware() {
        if(update) {
            switch (state) {
                case STOPPED:
                    robot.intake.setPower(0);
                    break;
                case FORWARD:
                    robot.intake.setPower(Common.INTAKE_FORWARD_POWER);
                    break;
                case BACKWARD:
                    robot.intake.setPower(Common.INTAKE_BACKWARD_POWER);
                    break;
            }
            update = false;
        }
    }
}
