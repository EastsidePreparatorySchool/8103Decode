package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Autonomous
public class LaserTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        // DigitalChannel pin0 = hardwareMap.digitalChannel.get("digital0");
        // DigitalChannel pin1 = hardwareMap.digitalChannel.get("digital1");
        AnalogInput pin0 = hardwareMap.analogInput.get("analog0");
        AnalogInput pin1 = hardwareMap.analogInput.get("analog1");

        waitForStart();

        while (opModeIsActive()) {
            // telemetry.addData("digital 0", pin0.getState());
            // telemetry.addData("digital 1", pin1.getState());
            telemetry.addData("analog 0", pin0.getVoltage());
            telemetry.addData("analog 1", pin1.getVoltage());
            telemetry.update();
        }
    }
}
