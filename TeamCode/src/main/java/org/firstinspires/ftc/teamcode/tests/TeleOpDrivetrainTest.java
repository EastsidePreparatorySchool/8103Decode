package org.firstinspires.ftc.teamcode.tests;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.DriveWithJoysticksCommand;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

@TeleOp(name = "TeleOpDrivetrainTest")
public class TeleOpDrivetrainTest extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private MecanumSubsystem mecanumSubsystem;

    @Override
    public void initialize() {
        robot.init(hardwareMap, telemetry);
        mecanumSubsystem = new MecanumSubsystem();
        register(mecanumSubsystem);

        DriveWithJoysticksCommand driveCommand = new DriveWithJoysticksCommand(
                mecanumSubsystem,
                () -> -gamepad1.left_stick_y,
                () -> gamepad1.left_stick_x,
                () -> gamepad1.right_stick_x,
                () -> gamepad1.left_trigger > 0.2
        );

        CommandScheduler.getInstance().setDefaultCommand(mecanumSubsystem, driveCommand);
        telemetry.addLine("Ready to drive");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("FL", mecanumSubsystem.getLastFL());
        telemetry.addData("FR", mecanumSubsystem.getLastFR());
        telemetry.addData("BL", mecanumSubsystem.getLastBL());
        telemetry.addData("BR", mecanumSubsystem.getLastBR());
        telemetry.update();
    }

    @Override
    public void end() {
        mecanumSubsystem.stop();
    }
}
