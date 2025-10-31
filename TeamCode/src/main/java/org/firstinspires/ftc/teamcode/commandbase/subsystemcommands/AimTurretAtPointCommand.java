package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.basecommands.TurretSetTargetCommand;

public class AimTurretAtPointCommand extends CommandBase {
    private final PinpointSubsystem odometry;
    private final TurretSubsystem turret;
    private volatile double targetX;
    private volatile double targetY;

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

    @Override
    public void execute() {
        double robotX = odometry.getPose().getX(DistanceUnit.INCH);
        double robotY = odometry.getPose().getY(DistanceUnit.INCH);
        double robotHeadingDeg = odometry.getPose().getHeading(AngleUnit.DEGREES);

        double targetX = this.targetX;
        double targetY = this.targetY;

        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        if (Math.hypot(deltaX, deltaY) < 1e-6) {
            return;
        }

        double fieldAngleDeg = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double turretTargetDeg = fieldAngleDeg - robotHeadingDeg;
        CommandScheduler.getInstance().schedule(new TurretSetTargetCommand(turret, turretTargetDeg));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
