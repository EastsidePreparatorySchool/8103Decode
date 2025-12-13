package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.PerpetualCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.AimTurretAtPointCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.AutoHoodPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.AutoShooterRPMCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.IntakeWhileFollowingPathCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.TripleShotCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.ShooterStateCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.TurretStateCommand;
import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.PersistentState;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Autonomous(name = "RightFarTwelveOnePreOneHPTwoGate", group = "Autos")
public class RightFarTwelveOnePreOneHPTwoGate extends CommandOpMode {
    private RobotHardware robot = RobotHardware.getInstance();
    private Follower follower;
    private CommandScheduler scheduler;
    private MultipleTelemetry multiTelemetry;
    private AimTurretAtPointCommand aimCommand;
    private AutoShooterRPMCommand shooterRPMCommand;
    private AutoHoodPositionCommand hoodPositionCommand;
    private PathChain path1and2, path3, path4, path5, path6to8, path9, path10to12, path13;

    @Override
    public void initialize() {
        scheduler = CommandScheduler.getInstance();
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createFollower(hardwareMap);
        robot.init(hardwareMap, multiTelemetry);
        robot.initLynx();
        robot.initDrivetrain();
        robot.initTurret();
        robot.initShooter();
        robot.initHood();
        robot.initIntake();
        robot.initTransfer();
        robot.initSpindexer();
        robot.initDistanceSensor();
        robot.initColorSensor();
        
        // Reset IMU on auto init for fresh calibration
        Common.PINPOINT_RESET_IMU_ON_INIT = true;
        robot.initPinpoint();
        
        // Set start position and target for right far auto
        Common.START_X_IN = Common.RIGHT_FAR_START_X_IN;
        Common.START_Y_IN = Common.RIGHT_FAR_START_Y_IN;
        Common.START_HEADING_DEG = Common.RIGHT_FAR_START_HEADING_DEG;
        Common.TARGET_X_IN = Common.RIGHT_FIELD_TARGET_X_IN;
        Common.TARGET_Y_IN = Common.RIGHT_FIELD_TARGET_Y_IN;
        Common.ACTUAL_TARGET_X_IN = Common.RIGHT_FIELD_ACTUAL_TARGET_X_IN;
        Common.ACTUAL_TARGET_Y_IN = Common.RIGHT_FIELD_ACTUAL_TARGET_Y_IN;
        
        aimCommand = new AimTurretAtPointCommand(Common.TARGET_X_IN, Common.TARGET_Y_IN);
        shooterRPMCommand = new AutoShooterRPMCommand(robot.shooterSubsystem);
        hoodPositionCommand = new AutoHoodPositionCommand(robot.hoodSubsystem);

        schedule(new TurretStateCommand(TurretSubsystem.TurretState.RUNNING));
        schedule(new ShooterStateCommand(ShooterSubsystem.ShooterState.OFF));

        scheduler.setDefaultCommand(robot.turretSubsystem, aimCommand);
        schedule(new PerpetualCommand(shooterRPMCommand));
        schedule(new PerpetualCommand(hoodPositionCommand));

        scheduler.run();

        buildPaths();
        schedule(
                new SequentialCommandGroup(
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.ON),
                        new TripleShotCommand(),
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.OFF),
                        // Intake while following path1and2 (ends when path done OR 3 balls collected)
                        new IntakeWhileFollowingPathCommand(follower, path1and2),
                        // Post-intake: turn on shooter, follow path3, tripleshot                        
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.ON),
                        new FollowPathCommand(follower, path3),
                        new TripleShotCommand(),
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.OFF),
                        // Intake while following path4 (ends when path done OR 3 balls collected)
                        new IntakeWhileFollowingPathCommand(follower, path4),
                        // Post-intake: turn on shooter, follow path5, tripleshot
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.ON),
                        new FollowPathCommand(follower, path5),
                        new TripleShotCommand(),
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.OFF),
                        // Intake while following path6to8 (ends when path done OR 3 balls collected)
                        new IntakeWhileFollowingPathCommand(follower, path6to8),
                        // Post-intake: turn on shooter, follow path9, tripleshot
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.ON),
                        new FollowPathCommand(follower, path9),
                        new TripleShotCommand(),
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.OFF)
                        // // Intake while following path10to12 (ends when path done OR 3 balls collected)
                        // new IntakeWhileFollowingPathCommand(follower, path10to12),
                        // // Post-intake: turn on shooter, follow path13, tripleshot
                        // new ShooterStateCommand(ShooterSubsystem.ShooterState.ON),
                        // new FollowPathCommand(follower, path13),
                        // new TripleShotCommand(),
                        // new ShooterStateCommand(ShooterSubsystem.ShooterState.OFF)
                )
        );
    }

    @Override
    public void initialize_loop() {
        robot.periodic();
        robot.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, Common.START_X_IN, Common.START_Y_IN, AngleUnit.DEGREES, Common.START_HEADING_DEG));
        multiTelemetry.addData("follower x", follower.getPose().getX());
        multiTelemetry.addData("follower y", follower.getPose().getY());
        multiTelemetry.addData("follower heading", Math.toDegrees(follower.getPose().getHeading()));
        multiTelemetry.update();
    }

    @Override
    public void run() {
        robot.periodic();
        follower.update();
        robot.robotX = follower.getPose().getX();
        robot.robotY = follower.getPose().getY();
        robot.robotHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        scheduler.run();

        multiTelemetry.addData("follower x", follower.getPose().getX());
        multiTelemetry.addData("follower y", follower.getPose().getY());
        multiTelemetry.addData("follower heading", Math.toDegrees(follower.getPose().getHeading()));
        double distance = Math.hypot(Common.TARGET_X_IN - robot.turretSubsystem.turretX,
                Common.TARGET_Y_IN - robot.turretSubsystem.turretY);
        multiTelemetry.addData("Distance to Goal", distance);
        multiTelemetry.addData("shooter rpm target", robot.shooterSubsystem.targetRpm);
        multiTelemetry.addData("shooter within tolerance", robot.shooterSubsystem.withinTolerance());
        multiTelemetry.addData("hood pos", robot.hoodSubsystem.hoodPos);
        multiTelemetry.update();
    }

    @Override
    public void end() {
        PersistentState.saveFromRobot();
    }

    public void buildPaths() {
        // Start point: (87, 8.25), heading starts at 90°
        Pose startPose = new Pose(87, 8.25, Math.toRadians(90));
        
        // Path 1 and 2 combined: Start → (103, 36) → (130, 36)
        path1and2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(87, 8.25),
                        new Pose(87, 36),
                        new Pose(103, 36)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(
                        new Pose(103, 36),
                        new Pose(130, 36)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 3: (130, 36) → (90, 11), constant heading at 0°
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(130, 36),
                        new Pose(90, 11)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 4: (90, 11) → (134, 8.5), constant heading at 0°
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(90, 11),
                        new Pose(134, 8.5)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 5: (134, 8.5) → (90, 11), constant heading at 0°
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(134, 8.5),
                        new Pose(90, 11)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Paths 6, 8a, 8b combined: (90, 11) → (135, 12) → (135, 20) → (135, 28)
        path6to8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(90, 11),
                        new Pose(135, 12)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Pose(135, 12),
                        new Pose(120, 16),
                        new Pose(135, 20)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Pose(135, 20),
                        new Pose(120, 24),
                        new Pose(135, 28)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 9: (135, 28) → (90, 11), constant heading at 0°
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(135, 28),
                        new Pose(90, 11)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Paths 10, 11, 12 combined: (90, 11) → (135, 12) → (135, 20) → (135, 28)
        path10to12 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(90, 11),
                        new Pose(135, 12)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Pose(135, 12),
                        new Pose(120, 16),
                        new Pose(135, 20)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Pose(135, 20),
                        new Pose(120, 24),
                        new Pose(135, 28)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 13: (135, 28) → (90, 11), constant heading at 0°
        path13 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(135, 28),
                        new Pose(90, 11)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
}
