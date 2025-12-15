package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.complexcommands.AntiPushDriveCommand;
import org.firstinspires.ftc.teamcode.commandbase.complexcommands.PositionHoldDriveCommand;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Tuning OpMode for Anti-Push system.
 * Use FTC Dashboard to adjust constants in real-time.
 * 
 * Displays:
 * - Actual vs expected velocities
 * - Correction values
 * - Position hold status
 * - Max velocities from robot constants
 */
@Config
@TeleOp(name = "Anti-Push Tuning", group = "Tests")
public class AntiPushTuning extends LinearOpMode {

    private RobotHardware robot;
    private MultipleTelemetry multiTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap);

        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        multiTelemetry.addLine("=== Anti-Push Tuning ===");
        multiTelemetry.addLine("All constants adjustable via FTC Dashboard");
        multiTelemetry.addLine("");
        multiTelemetry.addData("Max Vel X (from Constants)", Constants.driveConstants.getXVelocity());
        multiTelemetry.addData("Max Vel Y (from Constants)", Constants.driveConstants.getYVelocity());
        multiTelemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update Pinpoint
            robot.pinpointSubsystem.periodic();

            // Get velocities
            double velX = robot.pinpointSubsystem.getVelXInPerSec();
            double velY = robot.pinpointSubsystem.getVelYInPerSec();
            double velH = Math.toDegrees(robot.pinpointSubsystem.getVelHeadingRadPerSec());

            // Get position
            double posX = robot.pinpointSubsystem.getXInches();
            double posY = robot.pinpointSubsystem.getYInches();
            double heading = robot.pinpointSubsystem.getHeadingDegrees();

            // Calculate expected velocity from gamepad
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_trigger - gamepad1.left_trigger;

            double expectedVelX = Math.pow(forward, 3) * AntiPushDriveCommand.MAX_VEL_X;
            double expectedVelY = Math.pow(strafe, 3) * AntiPushDriveCommand.MAX_VEL_Y;
            double expectedVelH = Math.pow(turn, 3) / 1.5 * Math.toDegrees(AntiPushDriveCommand.MAX_OMEGA);

            double inputMag = Math.hypot(forward, strafe) + Math.abs(turn);
            boolean isIdle = inputMag < PositionHoldDriveCommand.INPUT_DEADBAND_ENTER;

            // Telemetry
            multiTelemetry.addLine("=== POSITION ===");
            multiTelemetry.addData("X (in)", "%.2f", posX);
            multiTelemetry.addData("Y (in)", "%.2f", posY);
            multiTelemetry.addData("Heading (deg)", "%.1f", heading);

            multiTelemetry.addLine("");
            multiTelemetry.addLine("=== VELOCITY ===");
            multiTelemetry.addData("Actual X (in/s)", "%.1f", velX);
            multiTelemetry.addData("Expected X (in/s)", "%.1f", expectedVelX);
            multiTelemetry.addData("Error X", "%.1f", expectedVelX - velX);
            multiTelemetry.addLine("");
            multiTelemetry.addData("Actual Y (in/s)", "%.1f", velY);
            multiTelemetry.addData("Expected Y (in/s)", "%.1f", expectedVelY);
            multiTelemetry.addData("Error Y", "%.1f", expectedVelY - velY);
            multiTelemetry.addLine("");
            multiTelemetry.addData("Actual H (deg/s)", "%.1f", velH);
            multiTelemetry.addData("Expected H (deg/s)", "%.1f", expectedVelH);

            multiTelemetry.addLine("");
            multiTelemetry.addLine("=== STATUS ===");
            multiTelemetry.addData("Input Magnitude", "%.3f", inputMag);
            multiTelemetry.addData("Is Idle", isIdle);
            multiTelemetry.addData("Vel Magnitude (in/s)", "%.1f", Math.hypot(velX, velY));

            multiTelemetry.addLine("");
            multiTelemetry.addLine("=== TUNABLE (Dashboard) ===");
            multiTelemetry.addData("KV_X", AntiPushDriveCommand.KV_X);
            multiTelemetry.addData("KV_Y", AntiPushDriveCommand.KV_Y);
            multiTelemetry.addData("KP_VEL", AntiPushDriveCommand.KP_VEL);
            multiTelemetry.addData("HOLD_KP", PositionHoldDriveCommand.HOLD_KP);

            multiTelemetry.update();

            // Simple drive for testing
            double multiplier = 0.7;
            double denominator = Math.max(1.0, Math.abs(forward) + Math.abs(strafe) + Math.abs(turn));
            double powerFL = multiplier * (forward + strafe + turn) / denominator;
            double powerFR = multiplier * (forward - strafe - turn) / denominator;
            double powerBL = multiplier * (forward - strafe + turn) / denominator;
            double powerBR = multiplier * (forward + strafe - turn) / denominator;

            robot.powerMotors(powerFL, powerFR, powerBL, powerBR);
        }
    }
}
