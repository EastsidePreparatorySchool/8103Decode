package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;

@Disabled
@Config
@TeleOp(name = "Stuff On Top Of The Turret", group = "Tuning")
public class OnTopOfTheTurret extends LinearOpMode {
    // Dashboard-tunable controls
    public static double flywheelPower = 0.0;
    public static double flywheel2Power = 0.0;
    public static double hoodPosition = 0.5;

    private final RobotHardware robot = RobotHardware.getInstance();
    private MultipleTelemetry multiTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, multiTelemetry);
        robot.initLynx();
        robot.initShooter();
        robot.initHood();

        // Defaults for this test
        flywheelPower = 0.0;
        flywheel2Power = 0.0;
        hoodPosition = 0.5;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            robot.periodic();

            double p1 = clamp(flywheelPower, -1.0, 1.0);
            double p2 = clamp(flywheel2Power, -1.0, 1.0);
            double hoodPos = clamp(hoodPosition, 0.0, 1.0);

            robot.flywheel.setPower(p1);
            if (robot.flywheel2 != null) {
                robot.flywheel2.setPower(p2);
            }
            robot.hood.setPosition(hoodPos);

            int ticks1 = robot.flywheel.getCurrentPosition();
            int ticks2 = (robot.flywheel2 != null) ? robot.flywheel2.getCurrentPosition() : 0;

            multiTelemetry.addData("flywheel/encoder", ticks1);
            multiTelemetry.addData("flywheel2/encoder", ticks2);
            multiTelemetry.addData("flywheel/power", p1);
            multiTelemetry.addData("flywheel2/power", p2);
            multiTelemetry.addData("hood/position", hoodPos);
            multiTelemetry.update();

            idle();
        }

        // Stop motors on exit
        robot.flywheel.setPower(0.0);
        if (robot.flywheel2 != null) {
            robot.flywheel2.setPower(0.0);
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}

