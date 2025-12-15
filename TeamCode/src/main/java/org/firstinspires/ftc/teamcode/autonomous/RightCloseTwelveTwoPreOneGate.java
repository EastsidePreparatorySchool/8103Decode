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
import com.seattlesolvers.solverslib.command.WaitCommand;
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

@Autonomous(name = "RightCloseTwelveTwoPreOneGate", group = "Autos")
public class RightCloseTwelveTwoPreOneGate extends CommandOpMode {
    private RobotHardware robot = RobotHardware.getInstance();
    private Follower follower;
    private CommandScheduler scheduler;
    private MultipleTelemetry multiTelemetry;
    private AimTurretAtPointCommand aimCommand;
    private AutoShooterRPMCommand shooterRPMCommand;
    private AutoHoodPositionCommand hoodPositionCommand;
    
    // PathChains
    private PathChain path1, path2, path3, path4, path4b, path5and6, path7, path8to11, path12;

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
        
        // Set start position for close auto
        Common.START_X_IN = Common.RIGHT_CLOSE_START_X_IN;
        Common.START_Y_IN = Common.RIGHT_CLOSE_START_Y_IN;
        Common.START_HEADING_DEG = Common.RIGHT_CLOSE_START_HEADING_DEG;
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
                        // Path 1 to shooting position, tripleshot
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.ON),
                        new SequentialCommandGroup(new FollowPathCommand(follower, path1), new WaitCommand(500)),
                        new TripleShotCommand(),
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.OFF),
                        // Path 2 intake
                        new IntakeWhileFollowingPathCommand(follower, path2),
                        // Path 3 (open gate)
                        new SequentialCommandGroup(new FollowPathCommand(follower, path3), new WaitCommand(500)),
                        // Path 4 to shooting, tripleshot
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.ON),
                        new SequentialCommandGroup(new FollowPathCommand(follower, path4), new WaitCommand(500)),
                        new TripleShotCommand(),
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.OFF),
                        // Path 4b (second gate open)
                        new SequentialCommandGroup(new FollowPathCommand(follower, path4b), new WaitCommand(500)),
                        // Paths 5+6 intake
                        new IntakeWhileFollowingPathCommand(follower, path5and6),
                        // Path 7 to shooting, tripleshot
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.ON),
                        new SequentialCommandGroup(new FollowPathCommand(follower, path7), new WaitCommand(500)),
                        new TripleShotCommand(),
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.OFF),
                        // Paths 8+9+10+11 intake
                        new IntakeWhileFollowingPathCommand(follower, path8to11),
                        // Path 12 to shooting, tripleshot
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.ON),
                        new SequentialCommandGroup(new FollowPathCommand(follower, path12), new WaitCommand(500)),
                        new TripleShotCommand(),
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.OFF)
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
        // Start point: (121, 120), heading 0°
        
        // Path 1: (121, 120) → (90, 84), linear 0° → 180°
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(121, 120),
                        new Pose(90, 84)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 2: (90, 84) → (130, 84), tangential heading
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(90, 84),
                        new Pose(130, 84)))
                .setTangentHeadingInterpolation()
                .build();

        // Path 3: (130, 84) → (130, 75), control (115, 81), linear 0° → 90°
        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(130, 84),
                        new Pose(115, 81),
                        new Pose(130, 75)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        // Path 4: (130, 75) → (90, 84), linear 90° → 0°
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(130, 75),
                        new Pose(90, 84)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        // Path 4b: (90, 84) → (130, 75), linear 0° → 90° (second gate open)
        path4b = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(90, 84),
                        new Pose(130, 75)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        // Paths 5+6 combined: (90, 84) → (100, 60) → (130, 60)
        path5and6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(90, 84),
                        new Pose(90, 60),
                        new Pose(98, 60)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(
                        new Pose(98, 60),
                        new Pose(130, 60)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 7: (130, 60) → (85, 75), constant 0°
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(130, 60),
                        new Pose(85, 75)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Paths 8+9+10+11 combined: (85, 75) → (130, 60) → (135, 60) → (135, 55) → (135, 50)
        path8to11 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(85, 75),
                        new Pose(90, 60),
                        new Pose(130, 60)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(
                        new Pose(130, 60),
                        new Pose(135, 60)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(
                        new Pose(135, 60),
                        new Pose(125, 57.5),
                        new Pose(135, 55)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Pose(135, 55),
                        new Pose(125, 52.5),
                        new Pose(135, 50)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 12: (135, 50) → (85, 75), constant 0°
        path12 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(135, 50),
                        new Pose(85, 75)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
}
