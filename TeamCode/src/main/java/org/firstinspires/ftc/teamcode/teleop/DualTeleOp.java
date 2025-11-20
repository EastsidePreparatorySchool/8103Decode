package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.safecommands.TurretSetTargetCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.AimTurretAtPointCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.TransferAndShootCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.TripleShotCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.HoodSetPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.PinpointSetPoseCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.ShooterSetTargetRPMCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.ShooterStateCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.SpindexerSetPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.TurretStateCommand;
import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.lib.PersistentState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name = "DualTeleOp", group = "Command")
@Config
public class DualTeleOp extends CommandOpMode {
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

    // Edge detection (gamepad2)
    private boolean prevA, prevBack, prevStart;
    // Gamepad1 A edge for intake toggle
    private boolean prevA1;
    private boolean prevY1;
    private boolean prevRB, prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight;
    private boolean prevStart1;
    private boolean prevLB2;

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
        // Maintain IMU heading if we have a saved pose
        Common.PINPOINT_RESET_IMU_ON_INIT = false;
        robot.initPinpoint();

        // Default commands
        driveCommand = new DriveWithGamepadCommand(gamepad1);
        aimCommand = new AimTurretAtPointCommand(Common.SELECTED_FIELD_TARGET_X_IN, Common.SELECTED_FIELD_TARGET_Y_IN);

        scheduler.setDefaultCommand(robot.mecanumSubsystem, driveCommand);
        scheduler.setDefaultCommand(robot.turretSubsystem, aimCommand);

        // Initial subsystem states
        schedule(new TurretStateCommand(TurretSubsystem.TurretState.RUNNING));
        schedule(new ShooterSetTargetRPMCommand(Common.SHOOTER_FAR_RPM));
        schedule(new ShooterStateCommand(ShooterSubsystem.ShooterState.OFF));
        schedule(new HoodSetPositionCommand(hoodPos));

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

        boolean shooterWithinTolerance = robot.shooterSubsystem.isAverageRpmWithinTolerance();

        // Keep aim target and offset updated each loop
        aimCommand.setTargetPoint(Common.SELECTED_FIELD_TARGET_X_IN, Common.SELECTED_FIELD_TARGET_Y_IN);
        aimCommand.setAngleOffsetDegrees(turretAngleOffsetDeg);

        // Gamepad2 right bumper: toggle shooter ON/OFF (target RPM set separately)
        boolean rb = gamepad2.right_bumper;
        if (rb && !prevRB) {
            ShooterSubsystem.ShooterState next = (robot.shooterSubsystem.state == ShooterSubsystem.ShooterState.ON)
                    ? ShooterSubsystem.ShooterState.OFF
                    : ShooterSubsystem.ShooterState.ON;
            schedule(new ShooterStateCommand(next));
        }
        prevRB = rb;

        // Gamepad1 A: toggle intake on/off
        boolean a1 = gamepad1.a;
        if (a1 && !prevA1) {
            IntakeSubsystem.IntakeState next = (robot.intakeSubsystem.state == IntakeSubsystem.IntakeState.FORWARD)
                    ? IntakeSubsystem.IntakeState.STOPPED
                    : IntakeSubsystem.IntakeState.FORWARD;
            schedule(new IntakeStateCommand(next));
        }
        prevA1 = a1;

        // Gamepad1 Y: cycle through intake positions 1 -> 2 -> 3 -> 1 ...
        boolean y1 = gamepad1.y;
        if (y1 && !prevY1) {
            int currIdx = intakeIndexFromState(robot.spindexerSubsystem.state);
            int nextIdx = (currIdx == -1) ? 0 : (currIdx + 1) % 3;
            schedule(new SpindexerSetPositionCommand(intakeStateForSlot(nextIdx)));
        }
        prevY1 = y1;

        // Gamepad2 left bumper: triple shot sequence (outtake slots 1,2,3)
        boolean lb2 = gamepad2.left_bumper;
        if (lb2 && !prevLB2) {
            if (shooterWithinTolerance && robot.shooterSubsystem.state == ShooterSubsystem.ShooterState.ON) {
                slotFull[0] = false;
                slotFull[1] = false;
                slotFull[2] = false;
                schedule(new TripleShotCommand());
            }
        }
        prevLB2 = lb2;

        // Gamepad2 Dpad up/down: adjust hood position by +/-0.01
        boolean dUp = gamepad2.dpad_up;
        boolean dDn = gamepad2.dpad_down;
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
        multiTelemetry.addData("hood pos", hoodPos);
        multiTelemetry.addData("turret offset (deg)", turretAngleOffsetDeg);
        multiTelemetry.addData("slot1 full", slotFull[0]);
        multiTelemetry.addData("slot2 full", slotFull[1]);
        multiTelemetry.addData("slot3 full", slotFull[2]);
        multiTelemetry.update();
    }

    // Slot button handlers removed; A cycles intake positions

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

    private static SpindexerSubsystem.SpindexerState outtakeStateForSlot(int slotIdx) {
        switch (slotIdx) {
            case 0:
                return SpindexerSubsystem.SpindexerState.OUTTAKE_ONE;
            case 1:
                return SpindexerSubsystem.SpindexerState.OUTTAKE_TWO;
            case 2:
                return SpindexerSubsystem.SpindexerState.OUTTAKE_THREE;
        }
        return SpindexerSubsystem.SpindexerState.OUTTAKE_ONE;
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

    private static int outtakeIndexFromState(SpindexerSubsystem.SpindexerState state) {
        switch (state) {
            case OUTTAKE_ONE:
                return 0;
            case OUTTAKE_TWO:
                return 1;
            case OUTTAKE_THREE:
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
