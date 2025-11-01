package org.firstinspires.ftc.teamcode.commandbase.subsystemcommands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.basecommands.MecanumPowerMotorsCommand;
import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;

public class DriveWithJoysticksCommand extends CommandBase {
    private final MecanumSubsystem mecanumSubsystem;
    private final Gamepad gamepad;

    public DriveWithJoysticksCommand(MecanumSubsystem mecanumSubsystem, Gamepad gamepad) {
        this.mecanumSubsystem = mecanumSubsystem;
        this.gamepad = gamepad;
        addRequirements(mecanumSubsystem);
    }

    public DriveWithJoysticksCommand(Gamepad gamepad) {
        this(
                RobotHardware.getInstance().mecanumSubsystem,
                gamepad
        );
    }

    @Override
    public void execute() {
        double forward = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = gamepad.right_stick_x;
        boolean slow = gamepad.left_trigger > 0.2;
        double multiplier = slow ? Common.SLOWMODE_MULTIPLIER : 1.0;
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);
        double powerFL = multiplier * (forward + strafe + turn) / denominator;
        double powerFR = multiplier * (forward - strafe - turn) / denominator;
        double powerBL = multiplier * (forward - strafe + turn) / denominator;
        double powerBR = multiplier * (forward + strafe - turn) / denominator;
        CommandScheduler.getInstance().schedule(
                new MecanumPowerMotorsCommand(
                        mecanumSubsystem,
                        powerFL,
                        powerFR,
                        powerBL,
                        powerBR
                )
        );
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
