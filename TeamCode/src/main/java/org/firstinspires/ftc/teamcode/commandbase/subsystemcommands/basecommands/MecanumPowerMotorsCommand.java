package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.basecommands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

import java.util.function.DoubleSupplier;

public class MecanumPowerMotorsCommand extends CommandBase {
    private final MecanumSubsystem mecanumSubsystem;
    private final DoubleSupplier powerFLSupplier;
    private final DoubleSupplier powerFRSupplier;
    private final DoubleSupplier powerBLSupplier;
    private final DoubleSupplier powerBRSupplier;

    public MecanumPowerMotorsCommand(MecanumSubsystem mecanumSubsystem,
                                     DoubleSupplier powerFLSupplier,
                                     DoubleSupplier powerFRSupplier,
                                     DoubleSupplier powerBLSupplier,
                                     DoubleSupplier powerBRSupplier) {
        this.mecanumSubsystem = mecanumSubsystem;
        this.powerFLSupplier = powerFLSupplier;
        this.powerFRSupplier = powerFRSupplier;
        this.powerBLSupplier = powerBLSupplier;
        this.powerBRSupplier = powerBRSupplier;
        addRequirements(mecanumSubsystem);
    }

    @Override
    public void initialize() {
        mecanumSubsystem.setMotorPowers(
                powerFLSupplier.getAsDouble(),
                powerFRSupplier.getAsDouble(),
                powerBLSupplier.getAsDouble(),
                powerBRSupplier.getAsDouble()
        );
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
