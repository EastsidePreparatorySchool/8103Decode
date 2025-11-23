package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "FarNinePieceAuto", group = "Command")
public class FarNinePieceAuto extends CommandOpMode {
    private RobotHardware robot = RobotHardware.getInstance();
    private Follower follower;
    private CommandScheduler scheduler;
    private MultipleTelemetry multiTelemetry;
    private AimTurretAtPointCommand aimCommand;
    private AutoShooterRPMCommand shooterRPMCommand;
    private AutoHoodPositionCommand hoodPositionCommand;
    private PathChain ball1Pickup, ball2Pickup, ball3Pickup, shot1, ball4Pickup, ball5Pickup, ball6Pickup, shot2;

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
                        new TripleShotCommand(),
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.OFF),
                        new IntakeStateCommand(IntakeSubsystem.IntakeState.FORWARD),
                        new FollowPathCommand(follower, ball1Pickup),
                        new WaitCommand(500),
                        new SpindexerSetPositionCommand(SpindexerSubsystem.SpindexerState.INTAKE_TWO),
                        new FollowPathCommand(follower, ball2Pickup),
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.ON),
                        new WaitCommand(500),
                        new SpindexerSetPositionCommand(SpindexerSubsystem.SpindexerState.INTAKE_THREE),
                        new FollowPathCommand(follower, ball3Pickup),
                        new WaitCommand(500),
                        new IntakeStateCommand(IntakeSubsystem.IntakeState.STOPPED),
                        new FollowPathCommand(follower, shot1),
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
                        new FollowPathCommand(follower, shot2),
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
        ball1Pickup = follower.pathBuilder()
                .addPath(new BezierCurve(AutoPoses.START, AutoPoses.BALL1_PICKUP_CONTROL1, AutoPoses.BALL1_PICKUP))
                .setLinearHeadingInterpolation(AutoPoses.START.getHeading(), AutoPoses.BALL1_PICKUP.getHeading())
                .build();
        ball2Pickup = follower.pathBuilder()
                .addPath(new BezierCurve(AutoPoses.BALL1_PICKUP, AutoPoses.BALL2_PICKUP))
                .setTangentHeadingInterpolation()
                .build();
        ball3Pickup = follower.pathBuilder()
                .addPath(new BezierCurve(AutoPoses.BALL2_PICKUP, AutoPoses.BALL3_PICKUP))
                .setTangentHeadingInterpolation()
                .build();
        shot1 = follower.pathBuilder()
                .addPath(new BezierCurve(AutoPoses.BALL3_PICKUP, AutoPoses.SHOT1))
                .setLinearHeadingInterpolation(AutoPoses.BALL3_PICKUP.getHeading(), AutoPoses.SHOT1.getHeading())
                .build();
        ball4Pickup = follower.pathBuilder()
                .addPath(new BezierCurve(AutoPoses.SHOT1, AutoPoses.BALL4_PICKUP_CONTROL1, AutoPoses.BALL4_PICKUP))
                .setLinearHeadingInterpolation(AutoPoses.SHOT1.getHeading(), AutoPoses.BALL4_PICKUP.getHeading())
                .build();
        ball5Pickup = follower.pathBuilder()
                .addPath(new BezierCurve(AutoPoses.BALL4_PICKUP, AutoPoses.BALL5_PICKUP))
                .setTangentHeadingInterpolation()
                .build();
        ball6Pickup = follower.pathBuilder()
                .addPath(new BezierCurve(AutoPoses.BALL5_PICKUP, AutoPoses.BALL6_PICKUP))
                .setTangentHeadingInterpolation()
                .build();
        shot2 = follower.pathBuilder()
                .addPath(new BezierCurve(AutoPoses.BALL5_PICKUP, AutoPoses.SHOT2))
                .setLinearHeadingInterpolation(AutoPoses.BALL5_PICKUP.getHeading(), AutoPoses.SHOT2.getHeading())
                .build();
    }
}
