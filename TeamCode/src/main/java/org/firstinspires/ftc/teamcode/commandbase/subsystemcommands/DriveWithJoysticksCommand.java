package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.basecommands.MecanumPowerMotorsCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveWithJoysticksCommand extends CommandBase {
    private final MecanumSubsystem mecanumSubsystem;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier turnSupplier;
    private final BooleanSupplier slowModeSupplier;
    private final MecanumPowerMotorsCommand powerCommand;
    private double desiredFL;
    private double desiredFR;
    private double desiredBL;
    private double desiredBR;

    public DriveWithJoysticksCommand(MecanumSubsystem mecanumSubsystem,
                                     DoubleSupplier forwardSupplier,
                                     DoubleSupplier strafeSupplier,
                                     DoubleSupplier turnSupplier,
                                     BooleanSupplier slowModeSupplier) {
        this.mecanumSubsystem = mecanumSubsystem;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.turnSupplier = turnSupplier;
        this.slowModeSupplier = slowModeSupplier;
        powerCommand = new MecanumPowerMotorsCommand(
                mecanumSubsystem,
                () -> desiredFL,
                () -> desiredFR,
                () -> desiredBL,
                () -> desiredBR
        );
        addRequirements(mecanumSubsystem);
    }

    @Override
    public void execute() {
        double forward = forwardSupplier.getAsDouble();
        double strafe = strafeSupplier.getAsDouble();
        double turn = turnSupplier.getAsDouble();
        boolean slow = slowModeSupplier.getAsBoolean();
        double multiplier = slow ? Common.SLOWMODE_MULTIPLIER : 1.0;
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);
        desiredFL = multiplier * (forward + strafe + turn) / denominator;
        desiredFR = multiplier * (forward - strafe - turn) / denominator;
        desiredBL = multiplier * (forward - strafe + turn) / denominator;
        desiredBR = multiplier * (forward + strafe - turn) / denominator;
        powerCommand.runNow();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        mecanumSubsystem.stop();
    }
}
