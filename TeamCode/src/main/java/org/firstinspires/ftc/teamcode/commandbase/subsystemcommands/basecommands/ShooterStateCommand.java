package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.basecommands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShooterStateCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final ShooterSubsystem.ShooterState shooterState;

    public ShooterStateCommand(ShooterSubsystem shooterSubsystem, ShooterSubsystem.ShooterState shooterState) {
        this.shooterSubsystem = shooterSubsystem;
        this.shooterState = shooterState;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setShooterState(shooterState);
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
