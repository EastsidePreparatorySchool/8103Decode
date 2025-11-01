package org.firstinspires.ftc.teamcode.commandbase.basecommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

public class MecanumPowerMotorsCommand extends InstantCommand {
    public MecanumPowerMotorsCommand(MecanumSubsystem mecanumSubsystem,
                                     double powerFL,
                                     double powerFR,
                                     double powerBL,
                                     double powerBR) {
        super(() -> mecanumSubsystem.setMotorPowers(powerFL, powerFR, powerBL, powerBR));
    }
}
