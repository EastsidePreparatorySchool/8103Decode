package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.DoubleSupplier;

public class AimTurretAtPointCommand extends CommandBase {
    private final PinpointSubsystem odometry;
    private final TurretSubsystem turret;
    private final DoubleSupplier targetXSupplier;
    private final DoubleSupplier targetYSupplier;

    public AimTurretAtPointCommand(PinpointSubsystem odometry,
                                   TurretSubsystem turret,
                                   DoubleSupplier targetXSupplier,
                                   DoubleSupplier targetYSupplier) {
        this.odometry = odometry;
        this.turret = turret;
        this.targetXSupplier = targetXSupplier;
        this.targetYSupplier = targetYSupplier;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        double robotX = odometry.getPose().getX(DistanceUnit.INCH);
        double robotY = odometry.getPose().getY(DistanceUnit.INCH);
        double robotHeadingDeg = odometry.getPose().getHeading(AngleUnit.DEGREES);

        double targetX = targetXSupplier.getAsDouble();
        double targetY = targetYSupplier.getAsDouble();

        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        if (Math.hypot(deltaX, deltaY) < 1e-6) {
            return;
        }

        double fieldAngleDeg = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double turretTargetDeg = fieldAngleDeg - robotHeadingDeg;
        turret.setTarget(turretTargetDeg);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
