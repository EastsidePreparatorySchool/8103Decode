package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.complexcommands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.PinpointSetPoseCommand;
import org.firstinspires.ftc.teamcode.lib.LoopRateAverager;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

@Disabled
@Config
@TeleOp(name = "PinpointDriveTest", group = "Tuning")
public class PinpointDriveTest extends CommandOpMode {
    public static double START_X_IN = 0.0;
    public static double START_Y_IN = 0.0;
    public static double START_HEADING_DEG = 0.0;

    private final RobotHardware robot = RobotHardware.getInstance();
    private CommandScheduler scheduler;
    private MultipleTelemetry multiTelemetry;
    private MecanumSubsystem mecanumSubsystem;
    private DriveWithGamepadCommand driveCommand;
    private final LoopRateAverager loopRate = new LoopRateAverager(50);

    @Override
    public void initialize() {
        scheduler = CommandScheduler.getInstance();
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware bring-up
        robot.init(hardwareMap, multiTelemetry);
        robot.initLynx();
        robot.initDrivetrain();
        robot.initPinpoint();

        mecanumSubsystem = robot.mecanumSubsystem;
        driveCommand = new DriveWithGamepadCommand(mecanumSubsystem, gamepad1);
        scheduler.setDefaultCommand(mecanumSubsystem, driveCommand);

        schedule(new PinpointSetPoseCommand(START_X_IN, START_Y_IN, START_HEADING_DEG));

        multiTelemetry.addLine("PinpointDriveTest ready: drive with gamepad1");
        multiTelemetry.update();
    }

    @Override
    public void run() {
        robot.periodic();
        scheduler.run();

        loopRate.update();
        multiTelemetry.addData("hz", loopRate.getHz());
        multiTelemetry.addData("x (in)", robot.pinpointSubsystem.getXInches());
        multiTelemetry.addData("y (in)", robot.pinpointSubsystem.getYInches());
        multiTelemetry.addData("heading (deg)", robot.pinpointSubsystem.getHeadingDegrees());
        multiTelemetry.update();
    }
}
