package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.complexcommands.AimTurretAtPointCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.TransferAndShootCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.HoodSetPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.PinpointSetPoseCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.ShooterSetTargetRPMCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.SpindexerSetPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.TurretStateCommand;
import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@TeleOp(name = "FullTeleOp", group = "Command")
public class FullTeleOp extends CommandOpMode {
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

    // Edge detection
    private boolean prevA, prevB, prevX, prevY, prevBack, prevStart;
    private boolean prevRSB, prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight;

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
        robot.initPinpoint();

        driveCommand = new DriveWithGamepadCommand(gamepad1);
        aimCommand = new AimTurretAtPointCommand(Common.FIELD_TARGET_X_IN, Common.FIELD_TARGET_Y_IN);

        scheduler.setDefaultCommand(robot.mecanumSubsystem, driveCommand);
        scheduler.setDefaultCommand(robot.turretSubsystem, aimCommand);

        schedule(new PinpointSetPoseCommand(Common.START_X_IN, Common.START_Y_IN, Common.START_HEADING_DEG));
        schedule(new TurretStateCommand(TurretSubsystem.TurretState.RUNNING));
        schedule(new HoodSetPositionCommand(hoodPos));
        schedule(new ShooterSetTargetRPMCommand(0.0));
    }

    @Override
    public void initialize_loop() {
        robot.periodic();
        // Keep aim target synced with dashboard constants during init
        aimCommand.setTargetPoint(Common.FIELD_TARGET_X_IN, Common.FIELD_TARGET_Y_IN);
        aimCommand.setAngleOffsetDegrees(turretAngleOffsetDeg);
        multiTelemetry.addData("aim target x (in)", Common.FIELD_TARGET_X_IN);
        multiTelemetry.addData("aim target y (in)", Common.FIELD_TARGET_Y_IN);
        multiTelemetry.update();
    }

    @Override
    public void run() {
        robot.periodic();
        scheduler.run();

        // Keep aim target and offset updated each loop
        aimCommand.setTargetPoint(Common.FIELD_TARGET_X_IN, Common.FIELD_TARGET_Y_IN);
        aimCommand.setAngleOffsetDegrees(turretAngleOffsetDeg);

        // Right stick button: toggle shooter target RPM between 0 and FAR
        boolean rsb = gamepad1.right_stick_button;
        if (rsb && !prevRSB) {
            double current = robot.shooterSubsystem.targetRpm;
            if (current > 0.0) {
                schedule(new ShooterSetTargetRPMCommand(0.0));
            } else {
                schedule(new ShooterSetTargetRPMCommand(Common.SHOOTER_FAR_RPM));
            }
        }
        prevRSB = rsb;

        // Back button: toggle intake on/off
        boolean back = gamepad1.back;
        if (back && !prevBack) {
            IntakeSubsystem.IntakeState next = (robot.intakeSubsystem.state == IntakeSubsystem.IntakeState.FORWARD)
                    ? IntakeSubsystem.IntakeState.STOPPED
                    : IntakeSubsystem.IntakeState.FORWARD;
            schedule(new IntakeStateCommand(next));
        }
        prevBack = back;

        // X/Y/B: select slot 1/2/3 and move to intake if empty, else outtake
        handleSlotButton(gamepad1.x, 0);
        handleSlotButton(gamepad1.y, 1);
        handleSlotButton(gamepad1.b, 2);

        // A: mark current intaking slot as full
        boolean a = gamepad1.a;
        if (a && !prevA) {
            SpindexerSubsystem.SpindexerState s = robot.spindexerSubsystem.state;
            int idx = intakeIndexFromState(s);
            if (idx != -1) {
                slotFull[idx] = true;
            }
        }
        prevA = a;

        // Start: Transfer and optionally mark current outtake slot empty if shooter running
        boolean start = gamepad1.start;
        if (start && !prevStart) {
            schedule(new TransferAndShootCommand(() -> {
                int idx = outtakeIndexFromState(robot.spindexerSubsystem.state);
                if (idx != -1) {
                    slotFull[idx] = false;
                }
            }));
        }
        prevStart = start;

        // Dpad up/down: adjust hood position by +/-0.01
        boolean dUp = gamepad1.dpad_up;
        boolean dDn = gamepad1.dpad_down;
        if (dUp && !prevDpadUp) {
            hoodPos = clamp01(hoodPos + 0.01);
            schedule(new HoodSetPositionCommand(hoodPos));
        }
        if (dDn && !prevDpadDown) {
            hoodPos = clamp01(hoodPos - 0.01);
            schedule(new HoodSetPositionCommand(hoodPos));
        }
        prevDpadUp = dUp;
        prevDpadDown = dDn;

        // Dpad left/right: adjust turret aim offset by 2 degrees
        boolean dLeft = gamepad1.dpad_left;
        boolean dRight = gamepad1.dpad_right;
        if (dLeft && !prevDpadLeft) {
            turretAngleOffsetDeg -= 2.0;
        }
        if (dRight && !prevDpadRight) {
            turretAngleOffsetDeg += 2.0;
        }
        prevDpadLeft = dLeft;
        prevDpadRight = dRight;

        // Telemetry summary
        multiTelemetry.addData("shooter rpm target", robot.shooterSubsystem.targetRpm);
        multiTelemetry.addData("hood pos", hoodPos);
        multiTelemetry.addData("turret offset (deg)", turretAngleOffsetDeg);
        multiTelemetry.addData("slot1 full", slotFull[0]);
        multiTelemetry.addData("slot2 full", slotFull[1]);
        multiTelemetry.addData("slot3 full", slotFull[2]);
        multiTelemetry.update();
    }

    private void handleSlotButton(boolean pressed, int slotIdx) {
        switch (slotIdx) {
            case 0:
                if (pressed && !prevX) onSlotPressed(slotIdx);
                prevX = pressed;
                break;
            case 1:
                if (pressed && !prevY) onSlotPressed(slotIdx);
                prevY = pressed;
                break;
            case 2:
                if (pressed && !prevB) onSlotPressed(slotIdx);
                prevB = pressed;
                break;
        }
    }

    private void onSlotPressed(int slotIdx) {
        boolean isFull = slotFull[slotIdx];
        SpindexerSubsystem.SpindexerState targetState = isFull
                ? outtakeStateForSlot(slotIdx)
                : intakeStateForSlot(slotIdx);
        schedule(new SpindexerSetPositionCommand(targetState));
    }

    private static SpindexerSubsystem.SpindexerState intakeStateForSlot(int slotIdx) {
        switch (slotIdx) {
            case 0: return SpindexerSubsystem.SpindexerState.INTAKE_ONE;
            case 1: return SpindexerSubsystem.SpindexerState.INTAKE_TWO;
            case 2: return SpindexerSubsystem.SpindexerState.INTAKE_THREE;
        }
        return SpindexerSubsystem.SpindexerState.INTAKE_ONE;
    }

    private static SpindexerSubsystem.SpindexerState outtakeStateForSlot(int slotIdx) {
        switch (slotIdx) {
            case 0: return SpindexerSubsystem.SpindexerState.OUTTAKE_ONE;
            case 1: return SpindexerSubsystem.SpindexerState.OUTTAKE_TWO;
            case 2: return SpindexerSubsystem.SpindexerState.OUTTAKE_THREE;
        }
        return SpindexerSubsystem.SpindexerState.OUTTAKE_ONE;
    }

    private static int intakeIndexFromState(SpindexerSubsystem.SpindexerState state) {
        switch (state) {
            case INTAKE_ONE: return 0;
            case INTAKE_TWO: return 1;
            case INTAKE_THREE: return 2;
            default: return -1;
        }
    }

    private static int outtakeIndexFromState(SpindexerSubsystem.SpindexerState state) {
        switch (state) {
            case OUTTAKE_ONE: return 0;
            case OUTTAKE_TWO: return 1;
            case OUTTAKE_THREE: return 2;
            default: return -1;
        }
    }

    private static double clamp01(double v) {
        if (v < 0.0) return 0.0;
        if (v > 1.0) return 1.0;
        return v;
    }
}

