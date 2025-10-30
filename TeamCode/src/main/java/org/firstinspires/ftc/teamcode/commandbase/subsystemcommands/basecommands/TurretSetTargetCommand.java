package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.basecommands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.function.DoubleSupplier;

public class TurretSetTargetCommand extends CommandBase {
    private final TurretSubsystem turretSubsystem;
    private final DoubleSupplier targetSupplier;

    public TurretSetTargetCommand(TurretSubsystem turretSubsystem, DoubleSupplier targetSupplier) {
        this.turretSubsystem = turretSubsystem;
        this.targetSupplier = targetSupplier;
        addRequirements(turretSubsystem);
    }

    public TurretSetTargetCommand(TurretSubsystem turretSubsystem, double targetDegrees) {
        this(turretSubsystem, () -> targetDegrees);
    }

    @Override
    public void initialize() {
        turretSubsystem.setTarget(targetSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public void runNow() {
        initialize();
        end(false);
    }
}
