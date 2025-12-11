package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Autonomous
public class LaserTest extends LinearOpMode {
    @Override
    public void runOpMode() {
         DigitalChannel pin0 = hardwareMap.digitalChannel.get("digital0");
         DigitalChannel pin1 = hardwareMap.digitalChannel.get("digital1");
//        AnalogInput pin0 = hardwareMap.analogInput.get("analog0");
//        AnalogInput pin1 = hardwareMap.analogInput.get("analog1");
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        waitForStart();

        while (opModeIsActive()) {
             multipleTelemetry.addData("digital 0", pin0.getState());
             multipleTelemetry.addData("digital 1", pin1.getState());
//            multipleTelemetry.addData("analog 0", pin0.getVoltage());
//            multipleTelemetry.addData("analog 1", pin1.getVoltage());
            multipleTelemetry.update();
        }
    }
}
