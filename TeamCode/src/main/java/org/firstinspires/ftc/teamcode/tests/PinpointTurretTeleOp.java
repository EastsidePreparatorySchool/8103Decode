package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.complexcommands.AimTurretAtPointCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.PinpointSetPoseCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.TurretStateCommand;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.lib.LoopRateAverager;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Disabled
@Config
@TeleOp(name = "PinpointTurretTeleOp", group = "Command")
public class PinpointTurretTeleOp extends CommandOpMode {
    public static double START_X_IN = 0.0;
    public static double START_Y_IN = 0.0;
    public static double START_HEADING_DEG = 0.0;
    public static double TARGET_X_IN = 0.0;
    public static double TARGET_Y_IN = 0.0;

    private final RobotHardware robot = RobotHardware.getInstance();
    private MultipleTelemetry multiTelemetry;
    private CommandScheduler scheduler;

    private AimTurretAtPointCommand aimCommand;
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
        robot.initTurret();
        robot.initPinpoint();

        // Default commands
        driveCommand = new DriveWithGamepadCommand(gamepad1);
        aimCommand = new AimTurretAtPointCommand(TARGET_X_IN, TARGET_Y_IN);

        scheduler.setDefaultCommand(robot.mecanumSubsystem, driveCommand);
        scheduler.setDefaultCommand(robot.turretSubsystem, aimCommand);

        schedule(new TurretStateCommand(TurretSubsystem.TurretState.RUNNING));
        schedule(new PinpointSetPoseCommand(START_X_IN, START_Y_IN, START_HEADING_DEG));
    }

    @Override
    public void initialize_loop() {
        robot.periodic();
        aimCommand.setTargetPoint(TARGET_X_IN, TARGET_Y_IN);
        multiTelemetry.addData("init target x (in)", TARGET_X_IN);
        multiTelemetry.addData("init target y (in)", TARGET_Y_IN);
        loopRate.update();
        multiTelemetry.addData("hz", loopRate.getHz());
        multiTelemetry.update();
    }

    @Override
    public void run() {
        robot.periodic();
        scheduler.run();
        aimCommand.setTargetPoint(TARGET_X_IN, TARGET_Y_IN);
        multiTelemetry.addData("target x (in)", TARGET_X_IN);
        multiTelemetry.addData("target y (in)", TARGET_Y_IN);
        multiTelemetry.addData("robot x (in)", robot.pinpointSubsystem.getXInches());
        multiTelemetry.addData("robot y (in)", robot.pinpointSubsystem.getYInches());
        multiTelemetry.addData("robot heading (deg)", robot.pinpointSubsystem.getHeadingDegrees());
        multiTelemetry.addData("turret target (deg)", robot.turretSubsystem.deg);
        loopRate.update();
        multiTelemetry.addData("hz", loopRate.getHz());
        multiTelemetry.update();
    }
}
