package org.firstinspires.ftc.teamcode.commandbase.complexcommands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commandbase.safecommands.TurretSetTargetCommand;
import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class AimTurretAtPointCommand extends CommandBase {
    private RobotHardware robot = RobotHardware.getInstance();
    private final TurretSubsystem turret;
    private volatile double targetX;
    private volatile double targetY;
    private volatile double angleOffsetDeg = 0.0;

    public AimTurretAtPointCommand(TurretSubsystem turret,
                                   double initialTargetX,
                                   double initialTargetY) {
        this.turret = turret;
        this.targetX = initialTargetX;
        this.targetY = initialTargetY;
        addRequirements(turret);
    }

    public AimTurretAtPointCommand(double initialTargetX, double initialTargetY) {
        this(
                RobotHardware.getInstance().turretSubsystem,
                initialTargetX,
                initialTargetY
        );
    }

    public void setTargetPoint(double targetX, double targetY) {
        this.targetX = targetX;
        this.targetY = targetY;
    }

    public void setAngleOffsetDegrees(double angleOffsetDeg) {
        this.angleOffsetDeg = angleOffsetDeg;
    }

    @Override
    public void execute() {
        double robotX = robot.robotX;
        double robotY = robot.robotY;
        double robotHeadingDeg = robot.robotHeadingDeg;
        double robotHeadingRad = Math.toRadians(robotHeadingDeg);

        double targetX = this.targetX;
        double targetY = this.targetY;

        // Compute turret world position so aiming accounts for turret offset from robot center
        double turretX = robotX
                + Common.TURRET_OFFSET_X_IN * Math.cos(robotHeadingRad);
        double turretY = robotY
                + Common.TURRET_OFFSET_X_IN * Math.sin(robotHeadingRad);
        turret.turretX = turretX;
        turret.turretY = turretY;

        double deltaX = targetX - turretX;
        double deltaY = targetY - turretY;
        if (Math.hypot(deltaX, deltaY) < 1e-6) {
            return;
        }

        double fieldAngleDeg = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double turretTargetDeg = fieldAngleDeg - robotHeadingDeg + angleOffsetDeg + 180;
        CommandScheduler.getInstance().schedule(new TurretSetTargetCommand(turret, turretTargetDeg));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
