package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.basecommands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretStateCommand extends CommandBase {
    private final TurretSubsystem turretSubsystem;
    private final TurretSubsystem.TurretState turretState;

    public TurretStateCommand(TurretSubsystem turretSubsystem, TurretSubsystem.TurretState turretState) {
        this.turretSubsystem = turretSubsystem;
        this.turretState = turretState;
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        turretSubsystem.setTurretState(turretState);
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
