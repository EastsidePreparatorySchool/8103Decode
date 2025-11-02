package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandbase.complexcommands.AimTurretAtPointCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.TransferAndShootCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.HoodSetPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.ShooterSetTargetRPMCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.SpindexerSetPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.TurretStateCommand;
import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.PersistentState;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Autonomous(name = "3PieceAuto", group = "Command")
public class ThreePieceAuto extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private CommandScheduler scheduler;
    private MultipleTelemetry multiTelemetry;

    private AimTurretAtPointCommand aimCommand;

    @Override
    public void initialize() {
        scheduler = CommandScheduler.getInstance();
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, multiTelemetry);
        robot.initLynx();
        robot.initDrivetrain();
        robot.initTurret();
        robot.initShooter();
        robot.initHood();
        robot.initIntake();
        robot.initTransfer();
        robot.initSpindexer();
        // Auto should reset IMU/odometry origin at start
        Common.PINPOINT_RESET_IMU_ON_INIT = true;
        robot.initPinpoint();

        aimCommand = new AimTurretAtPointCommand(Common.SELECTED_FIELD_TARGET_X_IN, Common.SELECTED_FIELD_TARGET_Y_IN);
        scheduler.setDefaultCommand(robot.turretSubsystem, aimCommand);

        schedule(new TurretStateCommand(TurretSubsystem.TurretState.RUNNING));

        // Build the 3-shot sequence
        SequentialCommandGroup threeShots = new SequentialCommandGroup(
                new HoodSetPositionCommand(Common.HOOD_FAR_POS),
                new ShooterSetTargetRPMCommand(Common.SHOOTER_FAR_RPM),

                new SpindexerSetPositionCommand(SpindexerSubsystem.SpindexerState.OUTTAKE_ONE),
                new WaitCommand(5000),
                new TransferAndShootCommand(() -> {}),
                new WaitCommand(5000),

                new SpindexerSetPositionCommand(SpindexerSubsystem.SpindexerState.OUTTAKE_TWO),
                new TransferAndShootCommand(() -> {}),
                new WaitCommand(5000),

                new SpindexerSetPositionCommand(SpindexerSubsystem.SpindexerState.OUTTAKE_THREE),
                new TransferAndShootCommand(() -> {}),
                new WaitCommand(2000),

                // Optional subclass-specific movement (e.g., strafe right/left)
                postShootingMovement(),

                // Before ending auto, point turret to 0 deg (straight ahead)
                new InstantCommand(() -> {
                    double x = robot.pinpointSubsystem.getXInches();
                    double y = robot.pinpointSubsystem.getYInches();
                    double headingRad = robot.pinpointSubsystem.getHeadingRadians();
                    double forward = 24.0; // inches
                    double targetX = x + forward * Math.cos(headingRad);
                    double targetY = y + forward * Math.sin(headingRad);
                    aimCommand.setTargetPoint(targetX, targetY);
                }),
                new WaitCommand(800),

                new ShooterSetTargetRPMCommand(0.0),
                new SpindexerSetPositionCommand(SpindexerSubsystem.SpindexerState.INTAKE_ONE),

                // Save odometry and turret data for teleop
                new InstantCommand(PersistentState::saveFromRobot)
        );

        schedule(threeShots);
    }

    // Hook for subclasses to inject movement between last shot and turret reset/save
    public SequentialCommandGroup postShootingMovement() {
        return new SequentialCommandGroup();
    }

    @Override
    public void run() {
        robot.periodic();
        scheduler.run();

        // Telemetry summary
        multiTelemetry.addData("pose x (in)", robot.pinpointSubsystem.getXInches());
        multiTelemetry.addData("pose y (in)", robot.pinpointSubsystem.getYInches());
        multiTelemetry.addData("heading (deg)", robot.pinpointSubsystem.getHeadingDegrees());
        multiTelemetry.addData("turret target (deg)", robot.turretSubsystem.deg);
        multiTelemetry.addData("shooter target rpm", robot.shooterSubsystem.targetRpm);
        multiTelemetry.update();
    }
}




