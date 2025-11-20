package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.complexcommands.AimTurretAtPointCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.TripleShotCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.HoodSetPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.PinpointSetPoseCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.ShooterSetTargetRPMCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.ShooterStateCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.SpindexerSetPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.TurretSetTargetCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.TurretStateCommand;
import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.lib.PersistentState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@TeleOp(name = "HoodShooterDataCollection", group = "Tuning")
@Config
public class HoodShooterDataCollection extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private CommandScheduler scheduler;
    private MultipleTelemetry multiTelemetry;

    // Default commands
    private AimTurretAtPointCommand aimCommand;
    private DriveWithGamepadCommand driveCommand;

    // Spindexer slot bookkeeping (1,2,3 => index 0..2)
    private final boolean[] slotFull = new boolean[] { false, false, false };

    // Hood and turret offsets
    private double hoodPos = Common.HOOD_FAR_POS;
    private double turretAngleOffsetDeg = 0.0;
    private double targetRpm = Common.SHOOTER_FAR_RPM;

    // Edge detection
    private boolean prevA, prevY, prevLB, prevRB, prevX;
    private boolean prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight;

    @Override
    public void initialize() {
        scheduler = CommandScheduler.getInstance();
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware bring-up
        robot.init(hardwareMap, multiTelemetry);
        robot.initLynx();
        robot.initDrivetrain();
        robot.initTurret();
        robot.initShooter();
        robot.initHood();
        robot.initIntake();
        robot.initTransfer();
        robot.initSpindexer();
        // Keep IMU heading if we have a saved pose from auto
        Common.PINPOINT_RESET_IMU_ON_INIT = false;
        robot.initPinpoint();

        // Default commands
        driveCommand = new DriveWithGamepadCommand(gamepad1);
        aimCommand = new AimTurretAtPointCommand(Common.SELECTED_FIELD_TARGET_X_IN, Common.SELECTED_FIELD_TARGET_Y_IN);

        scheduler.setDefaultCommand(robot.mecanumSubsystem, driveCommand);
        scheduler.setDefaultCommand(robot.turretSubsystem, aimCommand);

        // Initial subsystem states
        schedule(new TurretStateCommand(TurretSubsystem.TurretState.RUNNING));
        schedule(new HoodSetPositionCommand(hoodPos));
        schedule(new ShooterSetTargetRPMCommand(0.0));
        schedule(new ShooterStateCommand(ShooterSubsystem.ShooterState.OFF));

        // Restore saved pose/turret targets when available
        if (PersistentState.hasSavedPose) {
            schedule(new PinpointSetPoseCommand(PersistentState.savedXInches, PersistentState.savedYInches,
                    PersistentState.savedHeadingDeg));
        } else {
            schedule(new PinpointSetPoseCommand(Common.START_X_IN, Common.START_Y_IN, Common.START_HEADING_DEG));
        }
        if (PersistentState.hasSavedTurret) {
            schedule(new TurretSetTargetCommand(PersistentState.savedTurretDegrees));
        }
    }

    @Override
    public void initialize_loop() {
        robot.periodic();
        // Keep aim target synced with dashboard constants during init
        aimCommand.setTargetPoint(Common.SELECTED_FIELD_TARGET_X_IN, Common.SELECTED_FIELD_TARGET_Y_IN);
        aimCommand.setAngleOffsetDegrees(turretAngleOffsetDeg);
        multiTelemetry.addData("aim target x (in)", Common.SELECTED_FIELD_TARGET_X_IN);
        multiTelemetry.addData("aim target y (in)", Common.SELECTED_FIELD_TARGET_Y_IN);
        multiTelemetry.update();
    }

    @Override
    public void run() {
        robot.periodic();
        scheduler.run();

        // Keep aim target and offset updated each loop
        aimCommand.setTargetPoint(Common.SELECTED_FIELD_TARGET_X_IN, Common.SELECTED_FIELD_TARGET_Y_IN);
        aimCommand.setAngleOffsetDegrees(turretAngleOffsetDeg);

        // Right bumper: toggle shooter ON/OFF (target RPM managed separately)
        boolean rb = gamepad1.right_bumper;
        if (rb && !prevRB) {
            ShooterSubsystem.ShooterState next = (robot.shooterSubsystem.state == ShooterSubsystem.ShooterState.ON)
                    ? ShooterSubsystem.ShooterState.OFF
                    : ShooterSubsystem.ShooterState.ON;
            schedule(new ShooterStateCommand(next));
            // Keep target RPM aligned with the requested state
            double rpmCommand = (next == ShooterSubsystem.ShooterState.ON) ? targetRpm : 0.0;
            schedule(new ShooterSetTargetRPMCommand(rpmCommand));
        }
        prevRB = rb;

        // A: toggle intake on/off
        boolean a = gamepad1.a;
        if (a && !prevA) {
            IntakeSubsystem.IntakeState next = (robot.intakeSubsystem.state == IntakeSubsystem.IntakeState.FORWARD)
                    ? IntakeSubsystem.IntakeState.STOPPED
                    : IntakeSubsystem.IntakeState.FORWARD;
            schedule(new IntakeStateCommand(next));
        }
        prevA = a;

        // Y: cycle through intake positions 1 -> 2 -> 3 -> 1 ...
        boolean y = gamepad1.y;
        if (y && !prevY) {
            int currIdx = intakeIndexFromState(robot.spindexerSubsystem.state);
            int nextIdx = (currIdx == -1) ? 0 : (currIdx + 1) % 3;
            schedule(new SpindexerSetPositionCommand(intakeStateForSlot(nextIdx)));
        }
        prevY = y;

        // Left bumper: triple shot sequence (outtake slots 1,2,3)
        boolean lb = gamepad1.left_bumper;
        if (lb && !prevLB) {
            if (robot.shooterSubsystem.targetRpm > 0.0) {
                slotFull[0] = false;
                slotFull[1] = false;
                slotFull[2] = false;
            }
            schedule(new TripleShotCommand());
        }
        prevLB = lb;

        // Dpad up/down: adjust hood position by +/-0.01
        boolean dUp = gamepad1.dpad_up;
        boolean dDn = gamepad1.dpad_down;
        if (dUp && !prevDpadUp) {
            hoodPos = clamp01(hoodPos - 0.01);
            schedule(new HoodSetPositionCommand(hoodPos));
        }
        if (dDn && !prevDpadDown) {
            hoodPos = clamp01(hoodPos + 0.01);
            schedule(new HoodSetPositionCommand(hoodPos));
        }
        prevDpadUp = dUp;
        prevDpadDown = dDn;

        // Dpad left/right: adjust shooter RPM by +/- 100
        boolean dLeft = gamepad1.dpad_left;
        boolean dRight = gamepad1.dpad_right;
        if (dLeft && !prevDpadLeft) {
            targetRpm -= 100.0;
            if (targetRpm < 0)
                targetRpm = 0;
            // If shooter is running, update it immediately
            if (robot.shooterSubsystem.targetRpm > 0.0) {
                schedule(new ShooterSetTargetRPMCommand(targetRpm));
            }
        }
        if (dRight && !prevDpadRight) {
            targetRpm += 100.0;
            // If shooter is running, update it immediately
            if (robot.shooterSubsystem.targetRpm > 0.0) {
                schedule(new ShooterSetTargetRPMCommand(targetRpm));
            }
        }
        prevDpadLeft = dLeft;
        prevDpadRight = dRight;

        // X: Print Data to Telemetry
        boolean x = gamepad1.x;
        if (x && !prevX) {
            double dist = Math.hypot(
                    Common.SELECTED_FIELD_TARGET_X_IN - robot.pinpointSubsystem.getXInches(),
                    Common.SELECTED_FIELD_TARGET_Y_IN - robot.pinpointSubsystem.getYInches());
            multiTelemetry.addLine("=== DATA POINT ===");
            multiTelemetry.addData("Distance", dist);
            multiTelemetry.addData("Hood Pos", hoodPos);
            multiTelemetry.addData("Shooter RPM", targetRpm);
            multiTelemetry.addLine("==================");
        }
        prevX = x;

        // Telemetry summary
        multiTelemetry.addData("pose x (in)", robot.pinpointSubsystem.getXInches());
        multiTelemetry.addData("pose y (in)", robot.pinpointSubsystem.getYInches());
        multiTelemetry.addData("heading (deg)", robot.pinpointSubsystem.getHeadingDegrees());
        multiTelemetry.addData("shooter rpm target", targetRpm);
        multiTelemetry.addData("shooter rpm actual", robot.shooterSubsystem.currentRpm);
        multiTelemetry.addData("hood pos", hoodPos);
        multiTelemetry.addData("turret offset (deg)", turretAngleOffsetDeg);

        double currentDist = Math.hypot(
                Common.SELECTED_FIELD_TARGET_X_IN - robot.pinpointSubsystem.getXInches(),
                Common.SELECTED_FIELD_TARGET_Y_IN - robot.pinpointSubsystem.getYInches());
        multiTelemetry.addData("Distance to Goal", currentDist);

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

    private static double clamp01(double v) {
        if (v < 0.0)
            return 0.0;
        if (v > 1.0)
            return 1.0;
        return v;
    }
}
