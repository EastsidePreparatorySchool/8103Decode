package org.firstinspires.ftc.teamcode.tests;

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
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.TripleShotCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.HoodSetPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.ShooterSetTargetRPMCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.ShooterStateCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.SpindexerSetPositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.TurretStateCommand;
import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
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
    private boolean prevStart;

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
        Common.PINPOINT_RESET_IMU_ON_INIT = true;
        robot.initPinpoint();

        // Default commands
        driveCommand = new DriveWithGamepadCommand(gamepad1);
        aimCommand = new AimTurretAtPointCommand(Common.TARGET_X_IN, Common.TARGET_Y_IN);

        scheduler.setDefaultCommand(robot.mecanumSubsystem, driveCommand);
        scheduler.setDefaultCommand(robot.turretSubsystem, aimCommand);

        // Initial subsystem states
        schedule(new TurretStateCommand(TurretSubsystem.TurretState.RUNNING));
        schedule(new HoodSetPositionCommand(hoodPos));
        schedule(new ShooterSetTargetRPMCommand(0.0));
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

        // Keep aim target and offset updated each loop
        aimCommand.setTargetPoint(Common.TARGET_X_IN, Common.TARGET_Y_IN);
        aimCommand.setAngleOffsetDegrees(turretAngleOffsetDeg);

        boolean start = gamepad2.start;
        if (start && !prevStart) {
            targetRpm = Common.shooterInterpLUT.get(
                    Math.hypot(
                            144 - robot.turretSubsystem.turretX,
                            144 - robot.turretSubsystem.turretY)
            );
            hoodPos = Common.hoodInterpLUT.get(
                    Math.hypot(
                            144 - robot.turretSubsystem.turretX,
                            144 - robot.turretSubsystem.turretY)
            );
            schedule(new ShooterSetTargetRPMCommand(targetRpm));
            schedule(new HoodSetPositionCommand(hoodPos));
        }
        
        if(gamepad2.back) {
            targetRpm = 5000;
            hoodPos = 0.5;
            schedule(new ShooterSetTargetRPMCommand(targetRpm));
            schedule(new HoodSetPositionCommand(hoodPos));
        }

        // Gamepad2 right bumper: toggle shooter ON/OFF (target RPM managed separately)
        boolean rb = gamepad2.right_bumper;
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
            if (robot.shooterSubsystem.targetRpm > 0.0) {
                slotFull[0] = false;
                slotFull[1] = false;
                slotFull[2] = false;
            }
            schedule(new TripleShotCommand());
        }
        prevLB = lb;

        // Gamepad2 Dpad up/down: adjust hood position by +/-0.01
        boolean dUp = gamepad2.dpad_up;
        boolean dDn = gamepad2.dpad_down;
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

        // Gamepad2 Dpad left/right: adjust shooter RPM by +/- 100
        boolean dLeft = gamepad2.dpad_left;
        boolean dRight = gamepad2.dpad_right;
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
                    144 - robot.turretSubsystem.turretX,
                    144 - robot.turretSubsystem.turretY);
            multiTelemetry.addLine(String.format("%f, %f, %f", dist, hoodPos, targetRpm));
        }
        prevX = x;

        // Telemetry summary
        multiTelemetry.addData("pose x (in)", robot.pinpointSubsystem.getXInches());
        multiTelemetry.addData("pose y (in)", robot.pinpointSubsystem.getYInches());
        multiTelemetry.addData("turret x (in)", robot.turretSubsystem.turretX);
        multiTelemetry.addData("turret y (in)", robot.turretSubsystem.turretY);
        multiTelemetry.addData("turret target (deg)", robot.turretSubsystem.deg);
        multiTelemetry.addData("turret actual (deg)", robot.turretSubsystem.ticksToDegrees(robot.turretSubsystem.pos));
        multiTelemetry.addData("heading (deg)", robot.pinpointSubsystem.getHeadingDegrees());
        multiTelemetry.addData("shooter rpm target", targetRpm);
        multiTelemetry.addData("shooter rpm actual", robot.shooterSubsystem.currentRpm);
        multiTelemetry.addData("hood pos", hoodPos);

        double currentDist = Math.hypot(
                144 - robot.turretSubsystem.turretX,
                144 - robot.turretSubsystem.turretY);
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
