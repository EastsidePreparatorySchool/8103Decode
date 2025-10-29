package org.firstinspires.ftc.teamcode.tests;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.lib.Common;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name="TeleOpDrivetrainTest")
public class TeleOpDrivetrainTest extends LinearOpMode {
    // initialize telemetry
    private ElapsedTime runtime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();
    @Override
    public void runOpMode() {

        // initialize the hardware map
        robot.init(hardwareMap, telemetry);

        // Wait for start
        waitForStart();
        runtime.reset();

        double flpower = 0;
        double frpower = 0;
        double blpower = 0;
        double brpower = 0;

        // multiplier for slow mode
        double multiplier;

        while (opModeIsActive()) {

            /*
             * CONTROLLER CONFIGURATION:
             * SL: Drive
             * SR: Turn
             * Dpad: Directional movement
             *
             * RB:
             * RT:
             *
             * LB:
             * LT:
             *
             * A:
             * B:
             * X:
             * Y:
             * */

            // Check for controller inputs
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // Dpad Drive
            if (gamepad1.dpad_right) x = -1;
            if (gamepad1.dpad_left) x = 1;
            if (gamepad1.dpad_up) y = 1;
            if (gamepad1.dpad_down) y = -1;

            // Slow Mode
            if(gamepad1.left_trigger > 0.2) {
                multiplier = Common.SLOWMODE_MULTIPLIER;
            } else {
                multiplier = 1;
            }
            double denominator = Math.max(multiplier * (Math.abs(y) + Math.abs(x) + Math.abs(turn)), 1);
            // Set Power
            flpower = (y + x + turn) / denominator;
            frpower = (y - x - turn) / denominator;
            blpower = (y - x + turn) / denominator;
            brpower = (y + x - turn) / denominator;
            robot.powerMotors(flpower, frpower, blpower, brpower);
            // Telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("FL", flpower);
            telemetry.addData("FR", frpower);
            telemetry.addData("BL", blpower);
            telemetry.addData("BR", brpower);
            telemetry.update();
        }
    }
}