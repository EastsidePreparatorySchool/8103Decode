package org.firstinspires.ftc.teamcode.commandbase.complexcommands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commandbase.safecommands.MecanumPowerMotorsCommand;
import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;

public class DriveWithGamepadCommand extends CommandBase {
    private final MecanumSubsystem mecanumSubsystem;
    private final Gamepad gamepad;

    public DriveWithGamepadCommand(MecanumSubsystem mecanumSubsystem, Gamepad gamepad) {
        this.mecanumSubsystem = mecanumSubsystem;
        this.gamepad = gamepad;
        addRequirements(mecanumSubsystem);
    }

    public DriveWithGamepadCommand(Gamepad gamepad) {
        this(
                RobotHardware.getInstance().mecanumSubsystem,
                gamepad
        );
    }

    @Override
    public void execute() {
        double forward = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = gamepad.right_trigger - gamepad.left_trigger; // trigger turning: right is +, left is -

        // Cubed control for finer low-end response
        forward = Math.pow(forward, 3);
        strafe = Math.pow(strafe, 3);
        turn = Math.pow(turn, 3) / 1.5; // reduce turning sensitivity a bit
        double multiplier = Common.DRIVE_DEFAULT_MULT;
        double denominator = Math.max(multiplier * (Math.abs(forward) + Math.abs(strafe) + Math.abs(turn)), 1.0);
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
