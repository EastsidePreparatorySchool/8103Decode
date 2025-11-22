package org.firstinspires.ftc.teamcode.commandbase.complexcommands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commandbase.safecommands.TurretSetTargetCommand;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;

public class AimTurretAtPointCommand extends CommandBase {
    private final PinpointSubsystem odometry;
    private final TurretSubsystem turret;
    private volatile double targetX;
    private volatile double targetY;
    private volatile double angleOffsetDeg = 0.0;

    public AimTurretAtPointCommand(PinpointSubsystem odometry,
                                   TurretSubsystem turret,
                                   double initialTargetX,
                                   double initialTargetY) {
        this.odometry = odometry;
        this.turret = turret;
        this.targetX = initialTargetX;
        this.targetY = initialTargetY;
        addRequirements(turret);
    }

    public AimTurretAtPointCommand(double initialTargetX, double initialTargetY) {
        this(
                RobotHardware.getInstance().pinpointSubsystem,
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
        double robotX = odometry.getPose().getX(DistanceUnit.INCH);
        double robotY = odometry.getPose().getY(DistanceUnit.INCH);
        double robotHeadingDeg = odometry.getPose().getHeading(AngleUnit.DEGREES);
        double robotHeadingRad = Math.toRadians(robotHeadingDeg);

        double targetX = this.targetX;
        double targetY = this.targetY;

        // Compute turret world position so aiming accounts for turret offset from robot center
        double turretX = robotX
                + Common.TURRET_OFFSET_X_IN * Math.cos(robotHeadingRad);
        double turretY = robotY
                + Common.TURRET_OFFSET_X_IN * Math.sin(robotHeadingRad);

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
