package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.AimTurretAtPointCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsystemcommands.DriveWithJoysticksCommand;
import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Config
@TeleOp(name = "PinpointTurretTeleOp", group = "Command")
public class PinpointTurretTeleOp extends CommandOpMode {
    public static double START_X_IN = Common.PINPOINT_START_X_IN;
    public static double START_Y_IN = Common.PINPOINT_START_Y_IN;
    public static double START_HEADING_DEG = Common.PINPOINT_START_HEADING_DEG;
    public static double TARGET_X_IN = Common.PINPOINT_TARGET_X_IN;
    public static double TARGET_Y_IN = Common.PINPOINT_TARGET_Y_IN;

    private final RobotHardware robot = RobotHardware.getInstance();
    private MultipleTelemetry multiTelemetry;

    private MecanumSubsystem mecanumSubsystem;
    private TurretSubsystem turretSubsystem;
    private PinpointSubsystem pinpointSubsystem;

    @Override
    public void initialize() {
        Common.PINPOINT_START_X_IN = START_X_IN;
        Common.PINPOINT_START_Y_IN = START_Y_IN;
        Common.PINPOINT_START_HEADING_DEG = START_HEADING_DEG;
        Common.PINPOINT_TARGET_X_IN = TARGET_X_IN;
        Common.PINPOINT_TARGET_Y_IN = TARGET_Y_IN;

        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, multiTelemetry);
        robot.initDrivetrain();
        robot.initTurret();
        robot.initPinpoint();

        mecanumSubsystem = robot.mecanumSubsystem;
        turretSubsystem = robot.turretSubsystem;
        pinpointSubsystem = robot.pinpointSubsystem;

        register(mecanumSubsystem, turretSubsystem, pinpointSubsystem);

        pinpointSubsystem.initializePose(Common.PINPOINT_START_X_IN, Common.PINPOINT_START_Y_IN, Common.PINPOINT_START_HEADING_DEG);
        turretSubsystem.setTurretState(TurretSubsystem.TurretState.STOPPED);

        DriveWithJoysticksCommand driveCommand = new DriveWithJoysticksCommand(
                mecanumSubsystem,
                () -> -gamepad1.left_stick_y,
                () -> gamepad1.left_stick_x,
                () -> gamepad1.right_stick_x,
                () -> gamepad1.left_trigger > 0.2
        );

        AimTurretAtPointCommand aimCommand = new AimTurretAtPointCommand(
                pinpointSubsystem,
                turretSubsystem,
                () -> TARGET_X_IN,
                () -> TARGET_Y_IN
        );

        CommandScheduler.getInstance().setDefaultCommand(mecanumSubsystem, driveCommand);
        CommandScheduler.getInstance().setDefaultCommand(turretSubsystem, aimCommand);

        schedule(new InstantCommand(() -> turretSubsystem.setTurretState(TurretSubsystem.TurretState.RUNNING)));
    }

    @Override
    public void initialize_loop() {
        Common.PINPOINT_TARGET_X_IN = TARGET_X_IN;
        Common.PINPOINT_TARGET_Y_IN = TARGET_Y_IN;
        pinpointSubsystem.periodic();
        multiTelemetry.addData("init target x (in)", TARGET_X_IN);
        multiTelemetry.addData("init target y (in)", TARGET_Y_IN);
        multiTelemetry.update();
    }

    @Override
    public void run() {
        super.run();
        Common.PINPOINT_TARGET_X_IN = TARGET_X_IN;
        Common.PINPOINT_TARGET_Y_IN = TARGET_Y_IN;
        multiTelemetry.addData("target x (in)", TARGET_X_IN);
        multiTelemetry.addData("target y (in)", TARGET_Y_IN);
        multiTelemetry.addData("robot x (in)", pinpointSubsystem.getXInches());
        multiTelemetry.addData("robot y (in)", pinpointSubsystem.getYInches());
        multiTelemetry.addData("robot heading (deg)", pinpointSubsystem.getHeadingDegrees());
        multiTelemetry.addData("turret target (deg)", turretSubsystem.deg);
        multiTelemetry.update();
    }

    @Override
    public void end() {
        mecanumSubsystem.stop();
        if (robot.turret != null) {
            robot.turret.setPower(0.0);
        }
    }
}
