package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.PerpetualCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.AimTurretAtPointCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.AutoHoodPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.AutoShooterRPMCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.TripleShotCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.ShooterStateCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.SpindexerSetPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.TurretStateCommand;
import org.firstinspires.ftc.teamcode.lib.AutoPoses;
import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.PersistentState;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class CloseNinePieceAuto extends CommandOpMode {
    private RobotHardware robot = RobotHardware.getInstance();
    private Follower follower;
    private CommandScheduler scheduler;
    private MultipleTelemetry multiTelemetry;
    private AimTurretAtPointCommand aimCommand;
    private AutoShooterRPMCommand shooterRPMCommand;
    private AutoHoodPositionCommand hoodPositionCommand;
    private PathChain preloadShotPath, ball7Pickup, ball8Pickup, ball9Pickup, shot2Path, ball4Pickup, ball5Pickup, ball6Pickup, shot3Path;

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
        robot.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
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
                        new FollowPathCommand(follower, preloadShotPath),
                        new TripleShotCommand(),
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.OFF),
                        new IntakeStateCommand(IntakeSubsystem.IntakeState.FORWARD),
                        new FollowPathCommand(follower, ball7Pickup),
                        new WaitCommand(500),
                        new SpindexerSetPositionCommand(SpindexerSubsystem.SpindexerState.INTAKE_TWO),
                        new FollowPathCommand(follower, ball8Pickup),
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.ON),
                        new WaitCommand(500),
                        new SpindexerSetPositionCommand(SpindexerSubsystem.SpindexerState.INTAKE_THREE),
                        new FollowPathCommand(follower, ball9Pickup),
                        new WaitCommand(500),
                        new IntakeStateCommand(IntakeSubsystem.IntakeState.STOPPED),
                        new FollowPathCommand(follower, shot2Path),
                        new WaitCommand(500),
                        new TripleShotCommand(),
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.OFF),
                        new IntakeStateCommand(IntakeSubsystem.IntakeState.FORWARD),
                        new FollowPathCommand(follower, ball4Pickup),
                        new WaitCommand(500),
                        new SpindexerSetPositionCommand(SpindexerSubsystem.SpindexerState.INTAKE_TWO),
                        new FollowPathCommand(follower, ball5Pickup),
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.ON),
                        new WaitCommand(500),
                        new SpindexerSetPositionCommand(SpindexerSubsystem.SpindexerState.INTAKE_THREE),
                        new FollowPathCommand(follower, ball6Pickup),
                        new WaitCommand(500),
                        new FollowPathCommand(follower, shot3Path),
                        new IntakeStateCommand(IntakeSubsystem.IntakeState.STOPPED),
                        new WaitCommand(500),
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
        preloadShotPath = follower.pathBuilder()
                .addPath(new BezierCurve(AutoPoses.START, AutoPoses.SHOT_CLOSE))
                .setLinearHeadingInterpolation(AutoPoses.START.getHeading(), AutoPoses.SHOT_CLOSE.getHeading())
                .build();
        ball7Pickup = follower.pathBuilder()
                .addPath(new BezierCurve(AutoPoses.SHOT_CLOSE, AutoPoses.BALL_7_PICKUP_CONTROL1, AutoPoses.BALL_7_PICKUP))
                .setLinearHeadingInterpolation(AutoPoses.SHOT_CLOSE.getHeading(), AutoPoses.BALL_7_PICKUP.getHeading())
                .build();
        ball8Pickup = follower.pathBuilder()
                .addPath(new BezierCurve(AutoPoses.BALL_7_PICKUP, AutoPoses.BALL_8_PICKUP))
                .setTangentHeadingInterpolation()
                .build();
        ball9Pickup = follower.pathBuilder()
                .addPath(new BezierCurve(AutoPoses.BALL_8_PICKUP, AutoPoses.BALL_9_PICKUP))
                .setTangentHeadingInterpolation()
                .build();
        shot2Path = follower.pathBuilder()
                .addPath(new BezierCurve(AutoPoses.BALL_9_PICKUP, AutoPoses.SHOT_CLOSE))
                .setLinearHeadingInterpolation(AutoPoses.BALL_9_PICKUP.getHeading(), AutoPoses.SHOT_CLOSE.getHeading())
                .build();
        ball4Pickup = follower.pathBuilder()
                .addPath(new BezierCurve(AutoPoses.SHOT_CLOSE, AutoPoses.BALL4_PICKUP_CONTROL1, AutoPoses.BALL4_PICKUP))
                .setLinearHeadingInterpolation(AutoPoses.SHOT_CLOSE.getHeading(), AutoPoses.BALL4_PICKUP.getHeading())
                .build();
        ball5Pickup = follower.pathBuilder()
                .addPath(new BezierCurve(AutoPoses.BALL4_PICKUP, AutoPoses.BALL5_PICKUP))
                .setTangentHeadingInterpolation()
                .build();
        ball6Pickup = follower.pathBuilder()
                .addPath(new BezierCurve(AutoPoses.BALL5_PICKUP, AutoPoses.BALL6_PICKUP))
                .setTangentHeadingInterpolation()
                .build();
        shot3Path = follower.pathBuilder()
                .addPath(new BezierCurve(AutoPoses.BALL6_PICKUP, AutoPoses.SHOT_CLOSE))
                .setLinearHeadingInterpolation(AutoPoses.BALL6_PICKUP.getHeading(), AutoPoses.SHOT_CLOSE.getHeading())
                .build();
    }
}
