package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Autonomous
public class LaserTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DigitalChannel pin0 = hardwareMap.digitalChannel.get("digital0");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("digital 0", pin0.getState());
            telemetry.update();
        }
    }
}
