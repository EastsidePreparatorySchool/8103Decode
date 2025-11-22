package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.AimTurretAtPointCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.AutoHoodPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.AutoShooterRPMCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.TripleShotCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.ShooterStateCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.SpindexerSetPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.TurretStateCommand;
import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@TeleOp(name = "AutoShooterTestOpMode", group = "Testing")
@Config
public class AutoShooterTeleOpTest extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private CommandScheduler scheduler;
    private MultipleTelemetry multiTelemetry;

    // Default commands
    private AimTurretAtPointCommand aimCommand;
    private DriveWithGamepadCommand driveCommand;
    private AutoShooterRPMCommand shooterRPMCommand;
    private AutoHoodPositionCommand hoodPositionCommand;

    // Spindexer slot bookkeeping (1,2,3 => index 0..2)
    private final boolean[] slotFull = new boolean[] { false, false, false };

    // Turret offset
    private double turretAngleOffsetDeg = 0.0;

    // Edge detection
    private boolean prevA, prevY, prevLB, prevRB;
    private boolean prevDpadLeft, prevDpadRight;

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
        // If we have a saved pose from auto, avoid resetting IMU so heading stays
        // consistent
        Common.PINPOINT_RESET_IMU_ON_INIT = false;
        robot.initPinpoint();

        driveCommand = new DriveWithGamepadCommand(gamepad1);
        aimCommand = new AimTurretAtPointCommand(Common.TARGET_X_IN, Common.TARGET_Y_IN);
        shooterRPMCommand = new AutoShooterRPMCommand(robot.shooterSubsystem, robot.pinpointSubsystem);
        hoodPositionCommand = new AutoHoodPositionCommand(robot.hoodSubsystem, robot.pinpointSubsystem);

        scheduler.setDefaultCommand(robot.mecanumSubsystem, driveCommand);
        scheduler.setDefaultCommand(robot.turretSubsystem, aimCommand);
        scheduler.setDefaultCommand(robot.shooterSubsystem, shooterRPMCommand);
        scheduler.setDefaultCommand(robot.hoodSubsystem, hoodPositionCommand);

        schedule(new TurretStateCommand(TurretSubsystem.TurretState.RUNNING));
        schedule(new ShooterStateCommand(ShooterSubsystem.ShooterState.OFF));
    }

    @Override
    public void initialize_loop() {
        robot.periodic();
        robot.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, Common.START_X_IN, Common.START_Y_IN, AngleUnit.DEGREES, Common.START_HEADING_DEG));
        // Keep aim target synced with dashboard constants during init
        aimCommand.setTargetPoint(Common.TARGET_X_IN, Common.TARGET_Y_IN);
        aimCommand.setAngleOffsetDegrees(turretAngleOffsetDeg);
        multiTelemetry.addData("aim target x (in)", Common.TARGET_X_IN);
        multiTelemetry.addData("aim target y (in)", Common.TARGET_Y_IN);
        multiTelemetry.update();
    }

    @Override
    public void run() {
        robot.periodic();
        scheduler.run();

        boolean shooterWithinTolerance = robot.shooterSubsystem.withinTolerance();

        // Keep aim target and offset updated each loop
        aimCommand.setTargetPoint(Common.TARGET_X_IN, Common.TARGET_Y_IN);
        aimCommand.setAngleOffsetDegrees(turretAngleOffsetDeg);

        // Gamepad1 A: toggle intake on/off
        boolean a = gamepad1.a;
        if (a && !prevA) {
            IntakeSubsystem.IntakeState next = (robot.intakeSubsystem.state == IntakeSubsystem.IntakeState.FORWARD)
                    ? IntakeSubsystem.IntakeState.STOPPED
                    : IntakeSubsystem.IntakeState.FORWARD;
            schedule(new IntakeStateCommand(next));
        }
        prevA = a;

        // Gamepad1 Y: cycle through intake positions 1 -> 2 -> 3 -> 1 ...
        boolean y = gamepad1.y;
        if (y && !prevY) {
            int currIdx = intakeIndexFromState(robot.spindexerSubsystem.state);
            int nextIdx = (currIdx == -1) ? 0 : (currIdx + 1) % 3;
            schedule(new SpindexerSetPositionCommand(intakeStateForSlot(nextIdx)));
        }
        prevY = y;

        // Gamepad2 left bumper: triple shot sequence (outtake slots 1,2,3)
        boolean lb = gamepad2.left_bumper;
        if (lb && !prevLB) {
            if (shooterWithinTolerance && robot.shooterSubsystem.state == ShooterSubsystem.ShooterState.ON) {
                slotFull[0] = false;
                slotFull[1] = false;
                slotFull[2] = false;
                schedule(new TripleShotCommand());
            }
        }
        prevLB = lb;

        // Gamepad2 right bumper: toggle shooter on/off
        boolean rb = gamepad2.right_bumper;
        if (rb && !prevRB) {
            ShooterSubsystem.ShooterState nextState = (robot.shooterSubsystem.state == ShooterSubsystem.ShooterState.ON)
                    ? ShooterSubsystem.ShooterState.OFF
                    : ShooterSubsystem.ShooterState.ON;
            schedule(new ShooterStateCommand(nextState));
        }
        prevRB = rb;

        // Gamepad2 Dpad left/right: adjust turret aim offset by 4 degrees
        boolean dLeft = gamepad2.dpad_left;
        boolean dRight = gamepad2.dpad_right;
        if (dLeft && !prevDpadLeft) {
            turretAngleOffsetDeg += 4.0;
        }
        if (dRight && !prevDpadRight) {
            turretAngleOffsetDeg -= 4.0;
        }
        prevDpadLeft = dLeft;
        prevDpadRight = dRight;

        // Telemetry summary
        multiTelemetry.addData("pose x (in)", robot.pinpointSubsystem.getXInches());
        multiTelemetry.addData("pose y (in)", robot.pinpointSubsystem.getYInches());
        multiTelemetry.addData("heading (deg)", robot.pinpointSubsystem.getHeadingDegrees());
        multiTelemetry.addData("shooter rpm target", robot.shooterSubsystem.targetRpm);
        multiTelemetry.addData("shooter within tolerance", shooterWithinTolerance);
        multiTelemetry.addData("hood pos", robot.hoodSubsystem.hoodPos);
        multiTelemetry.addData("turret offset (deg)", turretAngleOffsetDeg);

        double distance = Math.hypot(Common.TARGET_X_IN - robot.pinpointSubsystem.getXInches(),
                Common.TARGET_Y_IN - robot.pinpointSubsystem.getYInches());
        multiTelemetry.addData("Distance to Goal", distance);

        // Turret debug
        multiTelemetry.addData("turret target (deg)", robot.turretSubsystem.deg);
        multiTelemetry.addData("turret actual (deg)", robot.turretSubsystem.ticksToDegrees(robot.turretSubsystem.pos));
        multiTelemetry.addData("turret power calc", robot.turretSubsystem.power);
        multiTelemetry.addData("slot1 full", slotFull[0]);
        multiTelemetry.addData("slot2 full", slotFull[1]);
        multiTelemetry.addData("slot3 full", slotFull[2]);
        multiTelemetry.update();
    }

    private static SpindexerSubsystem.SpindexerState intakeStateForSlot(int slotIdx) {
        switch (slotIdx) {
            case 0:
                return SpindexerSubsystem.SpindexerState.INTAKE_ONE;
            case 1:
                return SpindexerSubsystem.SpindexerState.INTAKE_TWO;
            case 2:
                return SpindexerSubsystem.SpindexerState.INTAKE_THREE;
        }
        return SpindexerSubsystem.SpindexerState.INTAKE_ONE;
    }

    private static int intakeIndexFromState(SpindexerSubsystem.SpindexerState state) {
        switch (state) {
            case INTAKE_ONE:
                return 0;
            case INTAKE_TWO:
                return 1;
            case INTAKE_THREE:
                return 2;
            default:
                return -1;
        }
    }
}
