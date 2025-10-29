package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;

public class TurretSubsystem extends SubsystemBase {
    public enum TurretState {
        STOPPED,
        RUNNING
    }

    private RobotHardware robot;
    public volatile TurretState state;
    public double deg; // turret should ALWAYS START facing forward, positive is counterclockwise relative to forward and negative is clockwise relative to forward as viewed from the top of the robot
    public PIDFController turretPIDF;
    public int pos;
    public int target;
    public double power;
    public TurretSubsystem() {
        robot = RobotHardware.getInstance();
        setTurretState(TurretState.STOPPED);
        turretPIDF = new PIDFController(Common.TURRET_KP, Common.TURRET_KI, Common.TURRET_KD, Common.TURRET_KF);
    }

    public void setTurretState(TurretState turretState) {
        state = turretState;
    }

    public void setTarget(double degrees) {
        deg = degrees;
        target = (int)deg; //TODO: some math here to convert the degrees to a position in ticks
    }

    public boolean withinTolerance() {
        return Math.abs(target-pos) < 10;
    }

    public void updateHardware() {
        pos = robot.turret.getCurrentPosition();
        power = MathUtils.clamp(turretPIDF.calculate(pos, target), -1, 1);
        robot.turret.setPower(power);
        robot.telemetry.addData("turret pos", pos);
        robot.telemetry.addData("turret target", target);
        robot.telemetry.addData("turret target (deg)", deg);
    }

    public void periodic() {
        if(state == TurretState.RUNNING) {
            updateHardware();
        }
    }
}