package org.firstinspires.ftc.teamcode.tests;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.complexcommands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.lib.LoopRateAverager;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

@TeleOp(name = "TeleOpDrivetrainTest", group = "Tuning")
public class TeleOpDrivetrainTest extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private CommandScheduler scheduler;
    private MecanumSubsystem mecanumSubsystem;
    private DriveWithGamepadCommand driveCommand;
    private final LoopRateAverager loopRate = new LoopRateAverager(50);

    @Override
    public void initialize() {
        scheduler = CommandScheduler.getInstance();
        robot.init(hardwareMap, telemetry);
        robot.initLynx();
        robot.initDrivetrain();
        mecanumSubsystem = robot.mecanumSubsystem;

        driveCommand = new DriveWithGamepadCommand(mecanumSubsystem, gamepad1);

        scheduler.setDefaultCommand(mecanumSubsystem, driveCommand);
        telemetry.addLine("Ready to drive");
        telemetry.update();
    }

    @Override
    public void run() {
        robot.periodic();
        scheduler.run();
        loopRate.update();
        telemetry.addData("hz", loopRate.getHz());
        telemetry.addData("FL", mecanumSubsystem.getLastFL());
        telemetry.addData("FR", mecanumSubsystem.getLastFR());
        telemetry.addData("BL", mecanumSubsystem.getLastBL());
        telemetry.addData("BR", mecanumSubsystem.getLastBR());
        telemetry.update();
    }
}
