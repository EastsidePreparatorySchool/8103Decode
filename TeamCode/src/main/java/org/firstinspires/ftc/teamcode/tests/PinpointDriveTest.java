package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.complexcommands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.PinpointSetPoseCommand;
import org.firstinspires.ftc.teamcode.lib.LoopRateAverager;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

@Config
@TeleOp(name = "PinpointDriveTest", group = "Command")
public class PinpointDriveTest extends CommandOpMode {
    public static double START_X_IN = 0.0;
    public static double START_Y_IN = 0.0;
    public static double START_HEADING_DEG = 0.0;

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
        robot.initPinpoint();

        mecanumSubsystem = robot.mecanumSubsystem;
        driveCommand = new DriveWithGamepadCommand(mecanumSubsystem, gamepad1);
        scheduler.setDefaultCommand(mecanumSubsystem, driveCommand);

        schedule(new PinpointSetPoseCommand(START_X_IN, START_Y_IN, START_HEADING_DEG));

        telemetry.addLine("PinpointDriveTest ready: drive with gamepad1");
        telemetry.update();
    }

    @Override
    public void run() {
        robot.periodic();
        scheduler.run();

        loopRate.update();
        telemetry.addData("hz", loopRate.getHz());
        telemetry.addData("x (in)", robot.pinpointSubsystem.getXInches());
        telemetry.addData("y (in)", robot.pinpointSubsystem.getYInches());
        telemetry.addData("heading (deg)", robot.pinpointSubsystem.getHeadingDegrees());
        telemetry.update();
    }
}
