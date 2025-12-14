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

@Autonomous(name = "LeftFarNineTwoPre", group = "Autos")
public class LeftFarNineTwoPre extends CommandOpMode {
    private RobotHardware robot = RobotHardware.getInstance();
    private Follower follower;
    private CommandScheduler scheduler;
    private MultipleTelemetry multiTelemetry;
    private AimTurretAtPointCommand aimCommand;
    private AutoShooterRPMCommand shooterRPMCommand;
    private AutoHoodPositionCommand hoodPositionCommand;
    
    // PathChains: First preset intake, return to shoot, second preset intake, return to shoot
    private PathChain intakePreset1, returnFromPreset1, intakePreset2, returnFromPreset2;

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
        
        // Set start position and target for left far auto
        Common.START_X_IN = Common.LEFT_FAR_START_X_IN;
        Common.START_Y_IN = Common.LEFT_FAR_START_Y_IN;
        Common.START_HEADING_DEG = Common.LEFT_FAR_START_HEADING_DEG;
        Common.TARGET_X_IN = Common.LEFT_FIELD_TARGET_X_IN;
        Common.TARGET_Y_IN = Common.LEFT_FIELD_TARGET_Y_IN;
        Common.ACTUAL_TARGET_X_IN = Common.LEFT_FIELD_ACTUAL_TARGET_X_IN;
        Common.ACTUAL_TARGET_Y_IN = Common.LEFT_FIELD_ACTUAL_TARGET_Y_IN;
        
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
                        // Tripleshot preload
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.ON),
                        new TripleShotCommand(),
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.OFF),
                        // Intake first preset (near start, like far twelve auto path1and2)
                        new IntakeWhileFollowingPathCommand(follower, intakePreset1),
                        // Return to shooting position and tripleshot
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.ON),
                        new SequentialCommandGroup(new FollowPathCommand(follower, returnFromPreset1), new WaitCommand(500)),
                        new TripleShotCommand(),
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.OFF),
                        // Intake second preset (like close auto path2 area, tangential intake)
                        new IntakeWhileFollowingPathCommand(follower, intakePreset2),
                        // Return to shooting position and tripleshot
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.ON),
                        new SequentialCommandGroup(new FollowPathCommand(follower, returnFromPreset2), new WaitCommand(500)),
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
        // Mirrored from right: X = 144 - rightX, heading 0° becomes 180°
        
        // First preset: Same pattern as far twelve auto path1and2, mirrored
        // (57, 8.25) → (41, 36) → (14, 36) with tangential intake
        intakePreset1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(57, 8.25),
                        new Pose(57, 36),
                        new Pose(46, 36)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(
                        new Pose(46, 36),
                        new Pose(14, 36)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Return from first preset to shooting position with linear heading to 90° for smooth transition
        // (14, 36) → (54, 11) linear interp from 180° to 90°
        returnFromPreset1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(14, 36),
                        new Pose(54, 11)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();

        // Second preset: Similar tangential pattern, intake at y=60 line (24 above first preset)
        // (54, 11) → curve up → (41, 60) → (14, 60) tangential intake
        intakePreset2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(54, 11),
                        new Pose(54, 60),
                        new Pose(46, 60)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(
                        new Pose(46, 60),
                        new Pose(14, 60)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Return from second preset to shooting position
        // (14, 60) → (54, 11)
        returnFromPreset2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(14, 60),
                        new Pose(54, 11)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
}
